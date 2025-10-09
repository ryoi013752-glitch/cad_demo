from controller import Robot
import cmath, math

def t1_ik(x, y): return math.degrees((2.0 * cmath.atan((1280.0*y + cmath.sqrt(-4000000.0*x**4 - 3200000.0*x**3 - 8000000.0*x**2*y**2 - 4000000.0*x**2*y + 178400.0*x**2 - 3200000.0*x*y**2 - 1600000.0*x*y + 327360.0*x - 4000000.0*y**4 - 4000000.0*y**3 - 181600.0*y**2 + 409200.0*y + 125911.0) + 320.0) / (2000.0*x**2 + 2080.0*x + 2000.0*y**2 + 1000.0*y + 461.0))).real)

def t2_ik(x, y): return math.degrees((2.0 * cmath.atan((1280.0*y - cmath.sqrt(-4000000.0*x**4 + 3200000.0*x**3 - 8000000.0*x**2*y**2 - 4000000.0*x**2*y + 178400.0*x**2 + 3200000.0*x*y**2 + 1600000.0*x*y - 327360.0*x - 4000000.0*y**4 - 4000000.0*y**3 - 181600.0*y**2 + 409200.0*y + 125911.0) + 320.0) / (2000.0*x**2 + 480.0*x + 2000.0*y**2 + 1000.0*y - 51.0))).real)

# 測試
#print(t1(0.2, 0.185), t2(0.2, 0.185))

# --- 校正參數 ---
# 初始的 t1 角度為垂直再向左轉 8.78 度
# 初始的 t 角度為垂直再向右轉 6.69 度
t1_offset = -8.78 - 90  # 以逆時針為正，t1 起始角度必須逆轉 8.78 度才能到垂直線, 再以水平線為 0 度則必須再逆轉 90度
t2_offset = 6.69 - 90 # 以逆時針位正, t2 起始角度必須正轉 6.69 度才能到垂直線,再以水平線為 0 度則必須再逆轉 90度
COMMON_OFFSET_RAD = math.pi / 2.0 # 往回轉 90 度, 表示要以水平線作為 0 度

# --- 機構參數 ---
L_VAL = 0.32
AX, AY = -0.2, -0.25
EX, EY = 0.2, -0.25

# --- 初始化 Webots ---
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

t1 = robot.getDevice("t1")
t2 = robot.getDevice("t2")

t1.setVelocity(5.0)
t2.setVelocity(5.0)

# 希望 x = 0.2, y = 0.185
# --- 設定目標角度（degree）---
'''
t1_deg = 0 + t1_offset
t2_deg = 0 + t2_offset
'''

t1_deg = t1_ik(0.2, 0.185) + t1_offset
t2_deg = t2_ik(0.2, 0.185) + t2_offset

# --- 轉換成 Webots 角度(radian)
t1_rad = math.radians(t1_deg)
t2_rad = math.radians(t2_deg)

t1_target = t1_rad
t2_target = t2_rad

# --- 發送目標角度給馬達 ---
t1.setPosition(t1_target)
t2.setPosition(t2_target)

# --- 持續運行直到結束（讓馬達能完成動作）---
while robot.step(TIME_STEP) != -1:
    pass

