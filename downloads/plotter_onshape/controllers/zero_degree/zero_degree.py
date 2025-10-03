from controller import Robot
import math

# 初始的 t1 角度為垂直再向左轉 8.78 度
# 初始的 t 角度為垂直再向右轉 6.69 度
t1_offset = -8.78 # 以逆時針為正，t1 起始角度必須逆轉 8.78 度才能到垂直 0 度
t2_offset = 6.69 # 以逆時針位正, t2 起始角度必須正轉 6.69 度才能到垂直 0 度
COMMON_OFFSET_RAD = math.pi / 2.0 # 往回轉 90 度, 表示要以水平線作為 0 度
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
#從這裡可以看出 joint 是以其初始位置作為 zero degree
t1_deg = 0 + t1_offset
t2_deg = 0 + t2_offset

# --- 轉換成 Webots 角度（radian），加上偏移與修正 ---
t1_rad = math.radians(t1_deg)
t2_rad = math.radians(t2_deg)

t1_target = t1_rad - COMMON_OFFSET_RAD # 若以水平線為 0 度, 則從前面垂直設為 0 度必須逆轉 90 度才到水平零度
t2_target = (t2_rad - COMMON_OFFSET_RAD)

# --- 發送目標角度給馬達 ---
t1.setPosition(t1_target)
t2.setPosition(t2_target)

# --- 持續運行直到結束（讓馬達能完成動作）---
while robot.step(TIME_STEP) != -1:
    pass
