"""my_controller controller."""

# 已經完成校正以及納入 IK 方程式
from controller import Robot
import math
import numpy as np

# ----------------- 固定參數 (米) -----------------
L1, L2, L3, L4, L5 = 0.20, 0.27, 0.27, 0.20, 0.20
Ex = L5  # E 點的 x 座標 (0.2)

def ik(cx: float, cy: float) -> tuple:
    """
    計算五連桿機械臂在 (+1, -1) 配置下的馬達角度 (t1, t2)。
    
    t1: 左臂 (AB) 角度，配置為向上 (sigma1 = +1)
    t2: 右臂 (ED) 角度，配置為向下 (sigma2 = -1)
    
    :param cx: 終端點 C 的 x 座標
    :param cy: 終端點 C 的 y 座標
    :return: (t1_rad, t2_rad) 角度，若無解則返回 (None, None)
    """
    
    # ------------------ 左側連桿 (A-B-C) 求解 t1 (sigma1 = +1) ------------------
    
    # 1. AC 向量長度的平方和長度
    L_AC_sq = cx**2 + cy**2
    L_AC = math.sqrt(L_AC_sq)
    
    # 2. 檢查工作空間可達性
    if L_AC > (L1 + L2) or L_AC < abs(L1 - L2) or L_AC == 0:
        return (None, None)

    # 3. 計算 AC 角度 (alpha)
    alpha = math.atan2(cy, cx)
    
    # 4. 利用餘弦定理計算 <BAC 角度 (beta1)
    cos_beta1 = (L1**2 + L_AC_sq - L2**2) / (2 * L1 * L_AC)
    
    # 處理浮點誤差，確保 cos 值在 [-1, 1] 範圍內
    cos_beta1 = np.clip(cos_beta1, -1.0, 1.0)
    beta1 = math.acos(cos_beta1)
    
    # 5. t1 = alpha + sigma1 * beta1
    sigma1 = 1
    t1 = alpha + sigma1 * beta1
    
    # ------------------ 右側連桿 (E-D-C) 求解 t2 (sigma2 = -1) ------------------
    
    # 1. EC 向量分量和長度的平方和長度
    dx, dy = cx - Ex, cy
    L_EC_sq = dx**2 + dy**2
    L_EC = math.sqrt(L_EC_sq)

    # 2. 檢查工作空間可達性
    if L_EC > (L4 + L3) or L_EC < abs(L4 - L3) or L_EC == 0:
        return (None, None)

    # 3. 計算 EC 角度 (gamma)
    gamma = math.atan2(dy, dx)
    
    # 4. 利用餘弦定理計算 <DEC 角度 (beta2)
    cos_beta2 = (L4**2 + L_EC_sq - L3**2) / (2 * L4 * L_EC)
    
    # 處理浮點誤差
    cos_beta2 = np.clip(cos_beta2, -1.0, 1.0)
    beta2 = math.acos(cos_beta2)
    
    # 5. t2 = gamma + sigma2 * beta2
    sigma2 = -1
    t2 = gamma + sigma2 * beta2
    
    # 將角度轉換為 [0, 2*pi] 範圍
    t1_rad = t1 % (2 * math.pi)
    t2_rad = t2 % (2 * math.pi)
    
    #return t1_rad, t2_rad # 傳回 radian
    return math.degrees(t1_rad), math.degrees(t2_rad)


# initial t1=90 deg, t2=17.9 deg
deg = math.pi/180
# offset 單位為角度
t1_offset = -90
t2_offset = -17.9
# plotter C 點座標
'''
cx = 0.2531
cy = 0.2940
#
cx = 0.2
cy = 0.4
#
cx = 0
cy = 0.4
'''
cx = 0
cy = 0.2
# ik 使用角度為單位
t1_deg, t2_deg = ik(cx, cy)
print(t1_deg, t2_deg)
t1_rad = (t1_deg + t1_offset)*deg
t2_rad = (t2_deg + t2_offset)*deg

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
t1 = robot.getDevice("t1")
t2 = robot.getDevice("t2")


t1.setVelocity(5.0)
t2.setVelocity(5.0)
t1.setPosition(t1_rad)
t2.setPosition(t2_rad)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
