# --------------------------------------------------------------
#  walk_5_step.py   （正確混合 Supervisor + Robot，手動算 CoM）
# --------------------------------------------------------------
from controller import Supervisor, Keyboard
import math

# ========================
# 初始化：建立 Supervisor 並取得 Robot 功能
# ========================
supervisor = Supervisor()           # 建立 Supervisor（有 getFromDef）
robot = supervisor                   # 同一物件！用 robot 控制馬達
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

# 馬達（用 robot 取得）
t1 = robot.getDevice('t1'); t2 = robot.getDevice('t2'); t3 = robot.getDevice('t3'); t4 = robot.getDevice('t4'); t5 = robot.getDevice('t5')
rt1 = robot.getDevice('rt1'); rt2 = robot.getDevice('rt2'); rt3 = robot.getDevice('rt3'); rt4 = robot.getDevice('rt4'); rt5 = robot.getDevice('rt5')
left_motors = [t1, t2, t3, t4, t5]
right_motors = [rt1, rt2, rt3, rt4, rt5]

# 感測器
t1_sensor = robot.getDevice('t1_sensor'); t2_sensor = robot.getDevice('t2_sensor')
t3_sensor = robot.getDevice('t3_sensor'); t4_sensor = robot.getDevice('t4_sensor'); t5_sensor = robot.getDevice('t5_sensor')
rt1_sensor = robot.getDevice('rt1_sensor'); rt2_sensor = robot.getDevice('rt2_sensor')
rt3_sensor = robot.getDevice('rt3_sensor'); rt4_sensor = robot.getDevice('rt4_sensor'); rt5_sensor = robot.getDevice('rt5_sensor')
left_sensors = [t1_sensor, t2_sensor, t3_sensor, t4_sensor, t5_sensor]
right_sensors = [rt1_sensor, rt2_sensor, rt3_sensor, rt4_sensor, rt5_sensor]

for sensor in left_sensors + right_sensors:
    sensor.enable(timestep)

# ========================
# 全域變數
# ========================
left_target_deg = [0, 0, 0, 0, 0]
right_target_deg = [0, 0, 0, 0, 0]
MOVE_TIME = 1.0
FAST_MOVE_TIME = 0.5
MARGIN = 1.1
TOLERANCE = 0.01
target_positions = [0.0] * 10

special_mode = None
prev_a_pressed = False

# 重心顯示
COM_DISPLAY_INTERVAL = 0.1
last_com_time = 0.0

# ========================
# 【手動計算 CoM】零件質量與 DEF 名稱
# ========================
PART_MASSES = {
    'torso': 5.0,
    'head': 1.0,
    'left_upper_leg': 1.5,
    'left_lower_leg': 1.0,
    'left_foot': 0.5,
    'right_upper_leg': 1.5,
    'right_lower_leg': 1.0,
    'right_foot': 0.5,
}

part_nodes = {}

# 取得所有 DEF 節點（使用 Supervisor 功能）
for name in PART_MASSES.keys():
    node = supervisor.getFromDef(name.upper())  # 必須用 supervisor
    if node is None:
        print(f"警告：找不到 DEF '{name.upper()}'，將忽略此零件")
    else:
        part_nodes[name] = node

# ========================
# 基本函數
# ========================
def degrees_to_radians(deg_list):
    return [math.radians(d) for d in deg_list]

def update_target_positions():
    global target_positions
    target_positions = degrees_to_radians(left_target_deg) + degrees_to_radians(right_target_deg)

def get_current_positions():
    return [s.getValue() for s in left_sensors + right_sensors]

def calculate_velocities(move_time):
    current = get_current_positions()
    velocities = []
    for i in range(10):
        delta = abs(target_positions[i] - current[i])
        vel = delta / move_time if delta > 1e-6 else 0.0
        velocities.append(vel * MARGIN)
    return velocities

def set_motor_targets(move_time=MOVE_TIME):
    update_target_positions()
    current = get_current_positions()
    velocities = calculate_velocities(move_time)
    for i, motor in enumerate(left_motors + right_motors):
        motor.setPosition(target_positions[i])
        motor.setVelocity(velocities[i])
    print(f"移動到: 左腿{[round(d,1) for d in left_target_deg]}, 右腿{[round(d,1) for d in right_target_deg]} (時間: {move_time}s)")

def stop_all_motors():
    for motor in left_motors + right_motors:
        motor.setVelocity(0.0)

def is_arrived():
    current = get_current_positions()
    return all(abs(current[i] - target_positions[i]) < TOLERANCE for i in range(10))

# ========================
# 【手動計算 CoM】
# ========================
def get_com():
    total_mass = 0.0
    com_x = com_y = com_z = 0.0

    for name, mass in PART_MASSES.items():
        node = part_nodes.get(name)
        if node is None:
            continue

        center = node.getCenterOfMass()  # 每個 Solid 都有自己的 CoM
        if center is None:
            continue

        com_x += center[0] * mass
        com_y += center[1] * mass
        com_z += center[2] * mass
        total_mass += mass

    if total_mass > 0:
        return round(com_x / total_mass, 3), round(com_y / total_mass, 3), round(com_z / total_mass, 3)
    return 0.0, 0.0, 0.0

# ========================
# 姿態函數
# ========================
def stand_pose():
    global left_target_deg, right_target_deg
    left_target_deg = [0, 0, 0, 0, 0]
    right_target_deg = [0, 0, 0, 0, 0]
    print("=== 強制站立：所有關節歸零 ===")

def squat_pose():
    global left_target_deg, right_target_deg
    left_target_deg = [0, 0, 55, -110, 55]
    right_target_deg = [0, 0, 55, -110, 55]
    print("=== 蹲姿 ===")

def special_a_left_pose():
    global left_target_deg
    cur = [round(math.degrees(s.getValue()), 1) for s in left_sensors]
    left_target_deg = [5, cur[1]-10, -60, 15, 5]
    print(f"=== 按 a: 左腳特殊動作 ===")

def special_a_right_pose():
    global right_target_deg
    cur = [round(math.degrees(s.getValue()), 1) for s in right_sensors]
    right_target_deg = [5, cur[1]-10, -60, 15, 5]
    print(f"=== 左腳完成 → 右腳跟隨 ===")

# ========================
# 單關節微調
# ========================
def adjust_joint(side, idx, delta):
    if side == 'left':
        left_target_deg[idx] += delta
    else:
        right_target_deg[idx] += delta

def handle_key_adjust(key):
    mapping = {
        ord('1'): ('left',0,1), ord('2'): ('left',1,1), ord('3'): ('left',2,1),
        ord('4'): ('left',3,1), ord('5'): ('left',4,1),
        ord('Q'): ('left',0,-1), ord('W'): ('left',1,-1), ord('E'): ('left',2,-1),
        ord('R'): ('left',3,-1), ord('T'): ('left',4,-1),
        ord('6'): ('right',0,1), ord('7'): ('right',1,1), ord('8'): ('right',2,1),
        ord('9'): ('right',3,1), ord('0'): ('right',4,1),
        ord('Y'): ('right',0,-1), ord('U'): ('right',1,-1), ord('I'): ('right',2,-1),
        ord('O'): ('right',3,-1), ord('P'): ('right',4,-1),
    }
    if key in mapping:
        side, idx, delta = mapping[key]
        adjust_joint(side, idx, delta)

# ========================
# 初始化
# ========================
robot.step(timestep)
stand_pose()
set_motor_targets()

print("\n" + "="*70)
print(" 雙足機器人控制面板（每按一次 a → 左腳 → 右腳 → 歸零）")
print("="*70)
print("s : 站立 | k : 蹲姿")
print("a : 左腳 → 右腳 → 歸零")
print("1-5 / q-w-e-r-t : 左腿 ±1°")
print("6-0 / y-u-i-o-p : 右腿 ±1°")
print("每 0.1 秒顯示重心 CoM (x, y, z)【手動計算】")
print("="*70 + "\n")

# ========================
# 主迴圈
# ========================
moving = False

while robot.step(timestep) != -1:
    now = robot.getTime()

    # === 每 0.1 秒顯示 CoM ===
    if now - last_com_time >= COM_DISPLAY_INTERVAL:
        x, y, z = get_com()
        print(f"CoM: ({x}, {y}, {z})")
        last_com_time = now

    # === 按鍵處理 ===
    key = keyboard.getKey()
    a_pressed = (key == ord('A') or key == ord('a'))

    if a_pressed and not prev_a_pressed and special_mode is None:
        print("檢測到 a 按下 → 啟動序列")
        special_a_left_pose()
        set_motor_targets(FAST_MOVE_TIME)
        moving = True
        special_mode = 'a_left'

    prev_a_pressed = a_pressed

    if key != -1 and not a_pressed:
        if special_mode is not None:
            print("其他按鍵中斷 a 序列")
            special_mode = None
            moving = False
            stop_all_motors()

        if key in [ord('S'), ord('s')]:
            stand_pose()
            set_motor_targets()
            moving = True
        elif key in [ord('K'), ord('k')]:
            squat_pose()
            set_motor_targets()
            moving = True
        else:
            handle_key_adjust(key)
            set_motor_targets()
            moving = True

    # === 動作完成 ===
    if moving and is_arrived():
        stop_all_motors()
        moving = False
        print("階段完成")

        if special_mode == 'a_left':
            print("左腳完成 → 啟動右腳")
            special_a_right_pose()
            set_motor_targets(FAST_MOVE_TIME)
            moving = True
            special_mode = 'a_right'
        elif special_mode == 'a_right':
            print("右腳完成 → 歸零")
            stand_pose()
            set_motor_targets(MOVE_TIME)
            moving = True
            special_mode = 'a_return'
        elif special_mode == 'a_return':
            print("序列結束")
            special_mode = None