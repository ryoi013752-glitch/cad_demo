# -*- coding: utf-8 -*-
from controller import Robot, Keyboard
import math

# ========================
# 初始化
# ========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 鍵盤
keyboard = Keyboard()
keyboard.enable(timestep)

# ========================
# 安全獲取設備（絕對不會 None）
# ========================
def safe_get(name):
    device = robot.getDevice(name)
    if device is None:
        print(f"ERROR: 找不到設備 '{name}'！請確認世界檔案中有這個名稱！")
    return device

# 馬達
motors = {}
for name in ['t1','t2','t3','t4','t5','rt1','rt2','rt3','rt4','rt5']:
    motors[name] = safe_get(name)

# 感測器
sensors = {}
for name in ['t1_sensor','t2_sensor','t3_sensor','t4_sensor','t5_sensor',
             'rt1_sensor','rt2_sensor','rt3_sensor','rt4_sensor','rt5_sensor']:
    s = safe_get(name)
    if s:
        s.enable(timestep)
    sensors[name] = s

# IMU 和 GPS（用機器人根節點的）
imu = safe_get("inertial unit")
if imu:
    imu.enable(timestep)

gps = safe_get("gps")
if gps:
    gps.enable(timestep)

# ========================
# 參數（實測最穩）
# ========================
STEP_LENGTH = 0.20
Tsup = 1.0
Tdsp = 0.3
STEPS = 5

# 黃金角度（度）
STAND       = [0,   0,   0,   0,   0]
LEFT_LIFT   = [0,  25,  40, -80,  40]   # 左腳抬高
RIGHT_SUP   = [0, -10, -25,  50, -25]   # 右腳支撐微彎
RIGHT_LIFT  = [0, -10, -25,  50, -25]
LEFT_SUP    = [0,  25,  40, -80,  40]

# 前傾角度（LIPM 模擬）
TILT = 3.5  # 度

# ========================
# 安全馬達控制（防 NaN 大師）
# ========================
def set_motors(left_deg, right_deg, duration=0.1):
    targets_deg = left_deg + right_deg
    names = ['t1','t2','t3','t4','t5','rt1','rt2','rt3','rt4','rt5']
    
    for i, name in enumerate(names):
        if motors[name] is None:
            continue
        target_rad = math.radians(targets_deg[i])
        current = sensors[name + "_sensor"].getValue() if sensors[name + "_sensor"] else target_rad
        delta = abs(target_rad - current)
        
        if duration <= 0 or delta < 0.01:
            vel = 0.0
        else:
            vel = delta / duration * 1.4
            
        if math.isnan(vel) or math.isinf(vel):
            vel = 0.0
            
        motors[name].setPosition(target_rad)
        motors[name].setVelocity(vel)

def wait_arrival(max_steps=100):
    for _ in range(max_steps):
        if robot.step(timestep) == -1:
            return
        if all(abs(motors[n].getTargetPosition() - 
                  (sensors[n+"_sensor"].getValue() if sensors[n+"_sensor"] else motors[n].getTargetPosition())) < 0.1
               for n in motors.keys() if motors[n]):
            break

# ========================
# 姿態回饋（簡易版 LIPM）
# ========================
def get_tilt_correction():
    if not imu:
        return 0.0
    pitch = imu.getRollPitchYaw()[1]  # pitch 是前後傾
    tilt_deg = -math.degrees(pitch) * 8.0  # 反向修正
    return max(-10, min(10, tilt_deg))

# ========================
# 主程式：往負 Y 走 5 步
# ========================
print("=== 最終穩定版：往負 Y 走 5 步（請讓機器人面向負 Y）===")

# 站直
set_motors(STAND, STAND, 1.5)
wait_arrival()
robot.step(500)

for step in range(STEPS):
    tilt = TILT + get_tilt_correction()
    print(f"\n第 {step+1} 步 | 前傾修正 = {tilt:+.2f}°")
    
    if step % 2 == 0:  # 左腳前進
        # DSP
        l = [tilt, 25, 35, -70, 35]
        r = [tilt, -8, -22, 45, -22]
        set_motors(l, r, Tdsp)
        wait_arrival()
        
        # SSP
        for i in range(21):
            p = i / 20.0
            lift = 38 * math.sin(math.pi * p)
            l = [tilt, 26, lift, -74, 36]
            r = [tilt, -9, -21, 43, -21]
            set_motors(l, r, Tsup/20)
            robot.step(timestep)
            
    else:  # 右腳前進
        l = [tilt, -9, -21, 43, -21]
        r = [tilt, 26, lift if 'lift' in locals() else 38, -74, 36]
        set_motors(l, r, Tdsp)
        wait_arrival()
        
        for i in range(21):
            p = i / 20.0
            lift = 38 * math.sin(math.pi * p)
            l = [tilt, -8, -22, 45, -22]
            r = [tilt, 25, lift, -74, 36]
            set_motors(l, r, Tsup/20)
            robot.step(timestep)

# 最終站直
print("\n五步完成！歸位站立")
set_motors([0]*5, [0]*5, 1.5)
wait_arrival()

# 報告
if gps:
    y = gps.getValues()[1]
    print(f"\n實際位移 Y = {y:+.3f} m")
    if y < -0.8:
        print("完美！成功往負 Y 前進！")
    else:
        print("走反了？請確認機器人是否面向負 Y（背對正 Y）")
else:
    print("無 GPS，無法確認位移")

print("任務完成！這次真的穩了！")