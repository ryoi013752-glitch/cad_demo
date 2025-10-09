"""my_controller controller."""

from controller import Robot
import math
import numpy as np

# ----------------- 固定參數 (米) -----------------
L1, L2, L3, L4, L5 = 0.20, 0.27, 0.27, 0.20, 0.20
Ex = L5  # E 點的 x 座標 (0.2)

def ik(cx: float, cy: float) -> tuple:
    """
    計算五連桿機械臂在 (+1, -1) 配置下的馬達角度 (t1, t2)，角度以 degree 回傳。
    """
    L_AC_sq = cx**2 + cy**2
    L_AC = math.sqrt(L_AC_sq)

    if L_AC > (L1 + L2) or L_AC < abs(L1 - L2) or L_AC == 0:
        return (None, None)

    alpha = math.atan2(cy, cx)
    cos_beta1 = (L1**2 + L_AC_sq - L2**2) / (2 * L1 * L_AC)
    cos_beta1 = np.clip(cos_beta1, -1.0, 1.0)
    beta1 = math.acos(cos_beta1)
    t1 = alpha + beta1

    dx, dy = cx - Ex, cy
    L_EC_sq = dx**2 + dy**2
    L_EC = math.sqrt(L_EC_sq)

    if L_EC > (L4 + L3) or L_EC < abs(L4 - L3) or L_EC == 0:
        return (None, None)

    gamma = math.atan2(dy, dx)
    cos_beta2 = (L4**2 + L_EC_sq - L3**2) / (2 * L4 * L_EC)
    cos_beta2 = np.clip(cos_beta2, -1.0, 1.0)
    beta2 = math.acos(cos_beta2)
    t2 = gamma - beta2

    t1_rad = t1 % (2 * math.pi)
    t2_rad = t2 % (2 * math.pi)

    return math.degrees(t1_rad), math.degrees(t2_rad)

# ------------------ 初始設定 ------------------
deg = math.pi / 180
t1_offset = -90  # degrees
t2_offset = -17.9  # degrees

# 建立 robot 實例
robot = Robot()
timestep = int(robot.getBasicTimeStep())
t1 = robot.getDevice("t1")
t2 = robot.getDevice("t2")

t1.setVelocity(50.0)
t2.setVelocity(50.0)

# ------------------ 畫圓參數 ------------------
center_x = 0.1
center_y = 0.3
radius = 0.1
steps = 25  # 越大越平滑

# 等候啟動
for _ in range(5):
    robot.step(timestep)

# ------------------ 畫圓主迴圈 ------------------
for i in range(steps):
    theta = 2 * math.pi * i / steps
    cx = center_x + radius * math.cos(theta)
    cy = center_y + radius * math.sin(theta)

    t1_deg, t2_deg = ik(cx, cy)
    if t1_deg is None or t2_deg is None:
        print(f"[Warning] Unreachable point: ({cx:.3f}, {cy:.3f})")
        continue

    t1_rad = (t1_deg + t1_offset) * deg
    t2_rad = (t2_deg + t2_offset) * deg

    t1.setPosition(t1_rad)
    t2.setPosition(t2_rad)

    for _ in range(5):  # 停留數步驟讓運動平順
        robot.step(timestep)

# ------------------ 結束 ------------------
print("畫圓完成")
while robot.step(timestep) != -1:
    pass
