from controller import Robot
import math

# --- 機構參數 ---
L_VAL = 0.32
AX, AY = -0.2, -0.25
EX, EY = 0.2, -0.25

# --- 校正參數 ---
COMMON_OFFSET_RAD = math.pi / 2.0
T2_PHYSICAL_SIGN = -1.0

# --- 初始化 Webots ---
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

t1 = robot.getDevice("t1")
t2 = robot.getDevice("t2")

t1.setVelocity(5.0)
t2.setVelocity(5.0)

# --- 設定目標角度（degree）---
t1_deg = 69.68
t2_deg = 43.03

# --- 轉換成 Webots 角度（radian），加上偏移與修正 ---
t1_rad = math.radians(t1_deg)
t2_rad = math.radians(t2_deg+90)

t1_target = t1_rad - COMMON_OFFSET_RAD
t2_target = (t2_rad - COMMON_OFFSET_RAD) * T2_PHYSICAL_SIGN

# --- 發送目標角度給馬達 ---
t1.setPosition(t1_target)
t2.setPosition(t2_target)

# --- 持續運行直到結束（讓馬達能完成動作）---
while robot.step(TIME_STEP) != -1:
    pass
