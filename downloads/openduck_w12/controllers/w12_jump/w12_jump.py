from controller import Robot, Keyboard
import math

TIME_STEP = 16
MAX_SPEED = 12.0

STEP_HEIGHT     = 0.009
LATERAL_SWING = 0.016
GAIT_PERIOD     = 0.48

KP_PITCH = 0.58 
KD_PITCH = 0.09
PITCH_OFFSET = math.radians(-0.8) 

KP_ROLL  = 0.90

def deg_to_rad(deg):
    return deg * math.pi / 180.0

class YourDuckFinalFix:
    def __init__(self):
        self.robot = Robot()
        self.kb = Keyboard()
        self.kb.enable(TIME_STEP)
        
        self.imu = self.robot.getDevice("inertial_unit")
        if self.imu: self.imu.enable(TIME_STEP)

        self.motors = {}
        for n in ["t1","t2","t3","t4","t5","rt1","rt2","rt3","rt4","rt5"]:
            m = self.robot.getDevice(n)
            m.setVelocity(MAX_SPEED)
            self.motors[n] = m

        # 零力矩站姿
        self.stand_deg = {
            "t1":0,   "rt1":0,
            "t2":0,   "rt2":0,
            "t3": 64, "rt3": 64,   
            "t4": -126,"rt4":-126, 
            "t5": 62, "rt5": 62,   
        }
        self.stand = {k: deg_to_rad(v) for k, v in self.stand_deg.items()}

        print("Openduck 初始化(蹲下→站起，約需 1.4 秒)")

        # 1. 快速移動到蹲低姿勢
        squat = self.stand.copy()
        squat["t3"] = squat["rt3"] = deg_to_rad(30)
        squat["t4"] = squat["rt4"] = deg_to_rad(-80)
        squat["t5"] = squat["rt5"] = deg_to_rad(40)
        for n in self.motors:
            self.motors[n].setPosition(squat[n])
            
        # 僅等待 10 步 (約 160ms) 讓電機開始動作
        for _ in range(10): self.robot.step(TIME_STEP) 

        # 2. 快速站起來（60 步插值，約 960ms）
        num_interpolation_points = 60 
        for p in range(num_interpolation_points + 1):
            alpha = p / num_interpolation_points
            for n in self.motors:
                pos = squat[n] * (1-alpha) + self.stand[n] * alpha
                self.motors[n].setPosition(pos)
            self.robot.step(TIME_STEP) # 每點只 step 一次

        # 3. 再鎖住 0.48 秒
        for _ in range(30): 
            for n in self.motors:
                self.motors[n].setPosition(self.stand[n])
            self.robot.step(TIME_STEP)

        print("Openduck 起始姿勢完成，按下向前鍵進行小步跳躍！")

        self.phase = 0.0
        self.prev_err = 0.0
        self.dt = TIME_STEP / 1000.0

    def balance(self, l, r):
        if not self.imu: return l, r
        roll, pitch, _ = self.imu.getRollPitchYaw()
        err = pitch - PITCH_OFFSET
        derr = (err - self.prev_err) / self.dt
        self.prev_err = err

        p_corr = KP_PITCH * err + KD_PITCH * derr
        r_corr = KP_ROLL * roll

        l[2] += p_corr; l[4] -= p_corr
        r[2] += p_corr; r[4] -= p_corr
        l[1] -= r_corr
        r[1] += r_corr
        return l, r

    def step(self):
        self.phase = (self.phase + self.dt / GAIT_PERIOD) % 1.0
        
        k = self.kb.getKey()
        speed = (1 if k == Keyboard.UP else 0) - (1 if k == Keyboard.DOWN else 0)
        turn  = (1 if k == Keyboard.LEFT else 0) - (1 if k == Keyboard.RIGHT else 0)

        if abs(speed) < 0.5 and abs(turn) < 0.5:
            l = [self.stand[f"t{i+1}"] for i in range(5)]
            r = [self.stand[f"rt{i+1}"] for i in range(5)]
            l, r = self.balance(l, r)
            for i in range(5):
                self.motors[f"t{i+1}"].setPosition(l[i])
                self.motors[f"rt{i+1}"].setPosition(r[i])
            return

        t = self.phase * 2 * math.pi

        lift_l = max(0, math.sin(t)) * STEP_HEIGHT
        lift_r = max(0, math.sin(t + math.pi)) * STEP_HEIGHT
        fwd_l  = math.cos(t) * 0.045 * speed
        fwd_r  = math.cos(t + math.pi) * 0.045 * speed
        turn_o = math.sin(t) * 0.015 * turn
        lat_l  = math.cos(t) * LATERAL_SWING
        lat_r  = -math.cos(t) * LATERAL_SWING

        ly = -0.060 + lat_l + lift_l*0.8
        lz =  0.048 + fwd_l + turn_o
        ry = -0.060 + lat_r + lift_r*0.8
        rz =  0.048 + fwd_r - turn_o

        # IK 係數配合 Openduck 尺寸調整
        def ik(y, z):
            dy = y + 0.060
            dz = z - 0.048
            a3 = 0.45 + 3.92*dz + 1.32*dy
            a4 = -0.90 - 7.78*dz - 3.00*dy
            a5 = 0.45 + 3.82*dz + 1.70*dy
            return [0,0,a3,a4,a5]

        la = ik(ly, lz)
        ra = ik(ry, rz)
        ra[1] = -ra[1]

        la, ra = self.balance(la, ra)

        for i,a in enumerate(la): self.motors[f"t{i+1}"].setPosition(a)
        for i,a in enumerate(ra): self.motors[f"rt{i+1}"].setPosition(a)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.step()

if __name__ == "__main__":
    YourDuckFinalFix().run()