from controller import Robot, Keyboard
import math

# ===================================================================
# 全域常數設定
# ===================================================================
TIME_STEP     = 16                  # Webots 模擬時間步長 (ms)，約 62.5Hz 控制頻率
MAX_SPEED     = 12.0                # 馬達最大速度 (rad/s)，用於快速到達目標角度

# 步態參數（經過實測調校的最佳小跳步態）
STEP_HEIGHT   = 0.009               # 單腳抬腿高度 (m)，太高會失衡
LATERAL_SWING = 0.016               # 左右擺動幅度 (m)，增加側向穩定性
GAIT_PERIOD   = 0.48                # 一個完整步態週期時間 (秒)，決定跳躍頻率

# 姿態平衡 PD 控制器參數（極其重要！決定能否站穩）
KP_PITCH = 0.58                     # Pitch 方向比例增益
KD_PITCH = 0.09                     # Pitch 方向微分增益
PITCH_OFFSET = math.radians(-0.8)   # 目標俯仰角偏移（鴨子重心偏後，需微前傾）

KP_ROLL  = 0.90                     # Roll 方向比例增益（左右平衡）

def deg_to_rad(deg):
    """角度轉弧度小工具"""
    return deg * math.pi / 180.0


class YourDuckFinalFix:
    """
    Openduck 最終穩定版本控制器
    特色：
    1. 開機先蹲下 → 緩慢站起 → 鎖定姿勢（避免直接站起摔倒）
    2. 極穩定的 PD 姿態平衡（IMU 回授）
    3. 類似波士頓動力的小跳步態（trot-like hopping）
    4. 鍵盤即時控制前進/後退/左右轉彎
    """

    def __init__(self):
        # ---------------------- 基本裝置初始化 ----------------------
        self.robot = Robot()
        self.kb = Keyboard()                  # 啟用鍵盤控制
        self.kb.enable(TIME_STEP)

        # IMU（慣性測量單元）用於取得目前 roll/pitch/yaw
        self.imu = self.robot.getDevice("inertial_unit")
        if self.imu: 
            self.imu.enable(TIME_STEP)

        # ---------------------- 馬達初始化 ----------------------
        self.motors = {}
        # 鴨子共有 10 個關節：左腿 t1~t5，右腿 rt1~rt5
        for n in ["t1","t2","t3","t4","t5","rt1","rt2","rt3","rt4","rt5"]:
            m = self.robot.getDevice(n)
            m.setVelocity(MAX_SPEED)      # 設定最大速度，讓馬達快速到達目標
            self.motors[n] = m

        # ---------------------- 零力矩站姿定義 ----------------------
        # 這組角度是「重心剛好落在兩腳支撐多邊形內」的靜態平衡姿勢
        self.stand_deg = {
            "t1":0,    "rt1":0,          # 髖關節 yaw（左右不動）
            "t2":0,    "rt2":0,          # 髖關節 roll（外展/內收）
            "t3": 64,  "rt3": 64,        # 髖關節 pitch（大腿後頂 64°）
            "t4": -126,"rt4":-126,       # 膝關節（彎曲 -126°）
            "t5": 62,  "rt5": 62,        # 踝關節（腳掌壓平 62°）
        }
        # 轉成弧度存起來，之後直接使用
        self.stand = {k: deg_to_rad(v) for k, v in self.stand_deg.items()}

        print("Openduckt 初始化(蹲下→站起，約7秒)")

        # ---------------------- 安全啟動流程（關鍵！） ----------------------
        # 直接用站姿啟動會因為重心瞬間變化而前傾摔倒
        # 所以先蹲很低 → 慢慢站起來 → 再鎖住幾秒讓物理引擎穩定
        squat = self.stand.copy()
        squat["t3"] = squat["rt3"] = deg_to_rad(30)    # 大腿往前
        squat["t4"] = squat["rt4"] = deg_to_rad(-80)   # 膝蓋微彎
        squat["t5"] = squat["rt5"] = deg_to_rad(40)    # 腳尖微翹
        for n in self.motors:
            self.motors[n].setPosition(squat[n])
        for _ in range(80): 
            self.robot.step(TIME_STEP)                # 等待蹲下穩定

        # 200 步線性插值慢慢站起來（約 3.2 秒）
        for p in range(201):
            alpha = p / 200.0
            for n in self.motors:
                pos = squat[n] * (1-alpha) + self.stand[n] * alpha
                self.motors[n].setPosition(pos)
            for _ in range(5): 
                self.robot.step(TIME_STEP)            # 每插值一步多跑幾次物理

        # 再鎖住站姿 4 秒（250 steps），確保完全穩定
        for _ in range(250):
            for n in self.motors:
                self.motors[n].setPosition(self.stand[n])
            self.robot.step(TIME_STEP)

        print("Openduck 起始姿勢完成，按下向上鍵進行小步跳躍！")

        # ---------------------- 步態變數初始化 ----------------------
        self.phase = 0.0          # 步態相位 (0~1)
        self.prev_err = 0.0       # 上一次 pitch 誤差（用於 KD）
        self.dt = TIME_STEP / 1000.0   # 時間步長（秒）

    # ===================================================================
    # 姿態平衡 PD 控制器（最核心的穩定機制）
    # ===================================================================
    def balance(self, l, r):
        """
        根據 IMU 讀數即時修正左右腿的關節角度，防止前後左右傾倒
        l, r 是長度為 5 的 list，對應 t1~t5, rt1~rt5 的目標角度
        """
        if not self.imu: 
            return l, r

        roll, pitch, _ = self.imu.getRollPitchYaw()

        # Pitch（前後）誤差與誤差變化率
        err  = pitch - PITCH_OFFSET                  # 目標是稍微前傾 -0.8°
        derr = (err - self.prev_err) / self.dt        # 微分項
        self.prev_err = err

        # PD 控制輸出
        p_corr = KP_PITCH * err + KD_PITCH * derr     # 前後修正量
        r_corr = KP_ROLL  * roll                      # 左右修正量

        # 將修正量加到相關關節
        # t3/rt3（大腿）、t4/rt4（膝蓋）主要影響前後平衡
        l[2] += p_corr;  l[4] -= p_corr
        r[2] += p_corr;  r[4] -= p_corr

        # t2/rt2（髖外展）主要影響左右平衡
        l[1] -= r_corr
        r[1] += r_corr

        return l, r

    # ===================================================================
    # 主步態控制迴圈
    # ===================================================================
    def step(self):
        # 更新步態相位（週期性 0~1）
        self.phase = (self.phase + self.dt / GAIT_PERIOD) % 1.0
        
        # ---------------------- 鍵盤輸入 ----------------------
        k = self.kb.getKey()
        speed = (1 if k == Keyboard.UP else 0) - (1 if k == Keyboard.DOWN else 0)
        turn  = (1 if k == Keyboard.LEFT else 0) - (1 if k == Keyboard.RIGHT else 0)

        # ---------------------- 靜止站立模式 ----------------------
        if abs(speed) < 0.5 and abs(turn) < 0.5:
            l = [self.stand[f"t{i+1}"] for i in range(5)]
            r = [self.stand[f"rt{i+1}"] for i in range(5)]
            l, r = self.balance(l, r)                 # 仍持續做姿態平衡
            for i in range(5):
                self.motors[f"t{i+1}"].setPosition(l[i])
                self.motors[f"rt{i+1}"].setPosition(r[i])
            return

        # ---------------------- 動態跳躍步態 ----------------------
        t = self.phase * 2 * math.pi                   # 轉成弧度當作 sin/cos 參數

        # 左右腳交替抬腿（類似對角小跑）
        lift_l = max(0, math.sin(t)) * STEP_HEIGHT
        lift_r = max(0, math.sin(t + math.pi)) * STEP_HEIGHT

        # 前進量（與速度按鍵成正比）
        fwd_l  = math.cos(t) * 0.045 * speed
        fwd_r  = math.cos(t + math.pi) * 0.045 * speed

        # 轉向（左右差動）
        turn_o = math.sin(t) * 0.015 * turn

        # 側向八字擺動（大幅提升穩定性！）
        lat_l  =  math.cos(t) * LATERAL_SWING
        lat_r  = -math.cos(t) * LATERAL_SWING

        # 計算左右腳尖在身體座標系的目標位置（y: 左右, z: 前後）
        ly = -0.060 + lat_l + lift_l*0.8               # 左腳 y (負值在左)
        lz =  0.048 + fwd_l + turn_o                   # 左腳 z
        ry = -0.060 + lat_r + lift_r*0.8               # 右腳 y
        rz =  0.048 + fwd_r - turn_o                   # 右腳 z

        # ---------------------- 簡化反向運動學 (IK) ----------------------
        # 針對 Openduck 腿長比例實測出的線性近似 IK
        # 輸入：腳尖在髖關節下的 (y, z) 相對位置
        # 輸出：t3, t4, t5 的角度（t1=t2=0）
        def ik(y, z):
            dy = y + 0.060                             # 補回髖關節偏移
            dz = z - 0.048
            a3 =  0.45 + 3.92*dz + 1.32*dy           # t3 (大腿)
            a4 = -0.90 - 7.78*dz - 3.00*dy           # t4 (膝蓋)
            a5 =  0.45 + 3.82*dz + 1.70*dy           # t5 (踝)
            return [0, 0, a3, a4, a5]                  # t1,t2 固定為 0

        la = ik(ly, lz)                                 # 左腿 IK
        ra = ik(ry, rz)                                 # 右腿 IK
        ra[1] = -ra[1]                                  # 右腿 t2 方向相反（鏡像）

        # 最後再加上姿態平衡修正
        la, ra = self.balance(la, ra)

        # 寫入馬達目標角度
        for i, a in enumerate(la): 
            self.motors[f"t{i+1}"].setPosition(a)
        for i, a in enumerate(ra): 
            self.motors[f"rt{i+1}"].setPosition(a)

    # ===================================================================
    # 主迴圈
    # ===================================================================
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.step()


# ===================================================================
# 程式進入點
# ===================================================================
if __name__ == "__main__":
    YourDuckFinalFix().run()