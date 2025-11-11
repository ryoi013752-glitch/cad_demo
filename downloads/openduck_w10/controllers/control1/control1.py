from controller import Robot, Keyboard
import math

# ========================
# 初始化機器人與基本設定
# ========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 初始化鍵盤
keyboard = Keyboard()
keyboard.enable(timestep)

# ========================
# 取得馬達與位置感測器
# ========================
# 左腿馬達
t1 = robot.getDevice('t1'); t2 = robot.getDevice('t2'); t3 = robot.getDevice('t3')
t4 = robot.getDevice('t4'); t5 = robot.getDevice('t5')
# 右腿馬達
rt1 = robot.getDevice('rt1'); rt2 = robot.getDevice('rt2'); rt3 = robot.getDevice('rt3')
rt4 = robot.getDevice('rt4'); rt5 = robot.getDevice('rt5')

# 位置感測器
t1_sensor = robot.getDevice('t1_sensor'); t2_sensor = robot.getDevice('t2_sensor')
t3_sensor = robot.getDevice('t3_sensor'); t4_sensor = robot.getDevice('t4_sensor')
t5_sensor = robot.getDevice('t5_sensor')
rt1_sensor = robot.getDevice('rt1_sensor'); rt2_sensor = robot.getDevice('rt2_sensor')
rt3_sensor = robot.getDevice('rt3_sensor'); rt4_sensor = robot.getDevice('rt4_sensor')
rt5_sensor = robot.getDevice('rt5_sensor')

# 啟用感測器
for sensor in [t1_sensor, t2_sensor, t3_sensor, t4_sensor, t5_sensor,
               rt1_sensor, rt2_sensor, rt3_sensor, rt4_sensor, rt5_sensor]:
    sensor.enable(timestep)

# 馬達與感測器列表
left_motors = [t1, t2, t3, t4, t5]
right_motors = [rt1, rt2, rt3, rt4, rt5]
left_sensors = [t1_sensor, t2_sensor, t3_sensor, t4_sensor, t5_sensor]
right_sensors = [rt1_sensor, rt2_sensor, rt3_sensor, rt4_sensor, rt5_sensor]

# ========================
# 姿態定義（度）
# ========================
STAND_DEGREES = [0, 0, 0, 0, 0]
SQUAT_DEGREES = [0, 0, 55, -110, 55]

# 目標角度（左右腿）
left_target_deg = STAND_DEGREES.copy()
right_target_deg = STAND_DEGREES.copy()

# 運動參數
MOVE_TIME = 1.0
FAST_MOVE_TIME = 0.6
MARGIN = 1.1
TOLERANCE = 0.01

# 轉換為弧度目標
def degrees_to_targets(deg_list):
    return [math.radians(d) for d in deg_list]

target_positions = [0.0] * 10
initial_positions = [0.0] * 10

# ========================
# 設定馬達到目標（帶速度限制）
# ========================
def set_targets(left_deg, right_deg, move_time=MOVE_TIME):
    global target_positions, initial_positions, left_target_deg, right_target_deg
    left_target_deg = left_deg[:]
    right_target_deg = right_deg[:]
    
    left_targets = degrees_to_targets(left_deg)
    right_targets = degrees_to_targets(right_deg)
    target_positions = left_targets + right_targets
    
    # 讀取當前位置
    current = [s.getValue() for s in left_sensors + right_sensors]
    initial_positions = current
    
    # 計算所需速度
    velocities = []
    for i in range(10):
        delta = abs(target_positions[i] - current[i])
        vel = delta / move_time if delta > 1e-6 else 0.0
        velocities.append(vel * MARGIN)
    
    # 設定馬達
    for i, motor in enumerate(left_motors + right_motors):
        motor.setPosition(target_positions[i])
        motor.setVelocity(velocities[i])
    
    print(f"移動 → 左: {left_deg}, 右: {right_deg} (時間: {move_time}s)")

# 檢查是否到達目標
def is_arrived():
    current = [s.getValue() for s in left_sensors + right_sensors]
    return all(abs(current[i] - target_positions[i]) < TOLERANCE for i in range(10))

# 停止所有馬達
def stop_motors():
    for motor in left_motors + right_motors:
        motor.setVelocity(0.0)

# ========================
# 姿態函數
# ========================
def stand_pose():
    set_targets(STAND_DEGREES, STAND_DEGREES, MOVE_TIME)
    print("=== 站直姿態 ===")

def squat_pose():
    set_targets(SQUAT_DEGREES, SQUAT_DEGREES, MOVE_TIME)
    print("=== 蹲下姿態 ===")

# ========================
# LIPM 三步行走序列
# ========================
def walk_three_steps():
    print("\n" + "="*50)
    print("   開始 LIPM 三步行走控制")
    print("="*50)

    steps = [
        # Step 1: 右腳支撐，左腳前擺
        ({"left": [10, 0, -40, 30, 10],   "right": [-5, 0, 10, -20, 10]},  0.8),  # 重心右移 + 左腳抬
        ({"left": [5,  0,  10, -20, 10],  "right": [-5, 0, 10, -20, 10]},  0.6),  # 左腳落地，重心前移

        # Step 2: 左腳支撐，右腳前擺
        ({"left": [5,  0,  10, -20, 10],  "right": [10, 0, -40, 30, 10]},   0.8),
        ({"left": [0,  0,  10, -20, 10],  "right": [5,  0,  10, -20, 10]},  0.6),

        # Step 3: 右腳支撐，左腳前擺
        ({"left": [10, 0, -40, 30, 10],   "right": [5,  0,  10, -20, 10]},  0.8),
        ({"left": [0,  0,  10, -20, 10],  "right": [0,  0,  10, -20, 10]},  0.6),
    ]

    for i, (pose, t) in enumerate(steps):
        leg = "左" if i % 2 == 0 else "右"
        print(f"\n--- 第 {i//2 + 1} 步：{leg}腳前擺 ---")
        set_targets(pose["left"], pose["right"], t)
        while robot.step(timestep) != -1 and not is_arrived():
            pass
        print(f"第 {i//2 + 1} 步 子動作 {i%2 + 1} 完成")

    # 最後回到站立
    print("\n=== 行走完成，回到站立 ===")
    stand_pose()
    while robot.step(timestep) != -1 and not is_arrived():
        pass
    stop_motors()
    print("三步行走完成！")

# ========================
# 初始化：站立
# ========================
robot.step(timestep)  # 讓感測器有初始值
stand_pose()

# ========================
# 主控制迴圈
# ========================
print("\n控制說明：")
print("  [S/s] 站直")
print("  [K/k] 蹲下")
print("  [X/x] 走三步 (LIPM 模式)")
print("-" * 40)

last_key = -1
walking = False

while robot.step(timestep) != -1:
    key = keyboard.getKey()

    # 避免重複觸發
    if key != -1 and key != last_key:
        if key == ord('S') or key == ord('s'):
            if not walking:
                stand_pose()
        elif key == ord('K') or key == ord('k'):
            if not walking:
                squat_pose()
        elif key == ord('X') or key == ord('x'):
            if not walking:
                walking = True
                walk_three_steps()
                walking = False

    last_key = key

    # 持續檢查是否到達目標（非行走時）
    if not walking:
        current = [s.getValue() for s in left_sensors + right_sensors]
        arrived = all(abs(current[i] - target_positions[i]) < TOLERANCE for i in range(10))
        if arrived:
            stop_motors()