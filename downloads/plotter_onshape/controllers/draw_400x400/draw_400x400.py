from controller import Robot
import numpy as np

# --- 機構參數 ---
L = 320.0
B = 400.0

# --- 逆運動學函數 ---
def t1_sol1(x, y):
    dist = np.sqrt(x**2 + y**2)
    if dist == 0:
        return 0.0
    val = (L**2 + dist**2) / (2 * L * dist)
    val = np.clip(val, -1.0, 1.0)
    return np.arctan2(y, x) - np.arccos(val)

def t2_sol1(x, y):
    dist = np.sqrt((B - x)**2 + y**2)
    if dist == 0:
        return 0.0
    val = (L**2 + dist**2) / (2 * L * dist)
    val = np.clip(val, -1.0, 1.0)
    return np.arctan2(y, B - x) + np.arccos(val)

def reachable(x, y):
    d1 = np.sqrt(x**2 + y**2)
    d2 = np.sqrt((B - x)**2 + y**2)
    # 兩邊距離都要小於 2*L 才可達
    return d1 <= 2*L and d2 <= 2*L

# --- 產生路徑，與原本一樣（正方形路徑） ---
def generate_path():
    t = np.linspace(0, 1, 400)
    x_path = np.piecewise(t,
        [t < 0.25, (t >= 0.25) & (t < 0.5), (t >= 0.5) & (t < 0.75), t >= 0.75],
        [lambda t: 1600 * t,
         lambda t: 400,
         lambda t: 400 - 1600 * (t - 0.5),
         lambda t: 0])
    
    y_path = np.piecewise(t,
        [t < 0.25, (t >= 0.25) & (t < 0.5), (t >= 0.5) & (t < 0.75), t >= 0.75],
        [lambda t: 30,
         lambda t: 1600 * (t - 0.25) + 30,
         lambda t: 430,
         lambda t: 430 - 1600 * (t - 0.75)])
    
    return list(zip(x_path, y_path))

# --- Webots Controller 主程式 ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

left_motor.setPosition(float('inf'))  # 設定馬達為速度控制模式
right_motor.setPosition(float('inf'))

left_motor.setVelocity(1.0)
right_motor.setVelocity(1.0)

path = generate_path()
step_index = 0
total_steps = len(path)

while robot.step(timestep) != -1:
    if step_index >= total_steps:
        # 路徑跑完了，停止馬達或重置
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break

    x, y = path[step_index]

    if not reachable(x, y):
        print(f"Point ({x:.1f}, {y:.1f}) unreachable, skipping step {step_index}")
        step_index += 1
        continue

    try:
        theta1 = t1_sol1(x, y)
        theta2 = t2_sol1(x, y)

        # 若還要檢查NaN可加這段：
        if np.isnan(theta1) or np.isnan(theta2):
            print(f"NaN detected at step {step_index}, skipping")
            step_index += 1
            continue

        left_motor.setPosition(theta1)
        right_motor.setPosition(theta2)

    except Exception as e:
        print(f"Error at step {step_index}: {e}")

    step_index += 1

# 若需要，結束前可加入清理程式
