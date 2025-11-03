from controller import Robot, Keyboard
import math

# ========================
# 初始化
# ========================

robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

# 馬達
t1 = robot.getDevice('t1'); t2 = robot.getDevice('t2'); t3 = robot.getDevice('t3'); t4 = robot.getDevice('t4'); t5 = robot.getDevice('t5')
rt1 = robot.getDevice('rt1'); rt2 = robot.getDevice('rt2'); rt3 = robot.getDevice('rt3'); rt4 = robot.getDevice('rt4'); rt5 = robot.getDevice('rt5')

left_motors = [t1, t2, t3, t4, t5]
right_motors = [rt1, rt2, rt3, rt4, rt5]

# 感測器
t1_sensor = robot.getDevice('t1_sensor'); t2_sensor = robot.getDevice('t2_sensor'); t3_sensor = robot.getDevice('t3_sensor'); t4_sensor = robot.getDevice('t4_sensor'); t5_sensor = robot.getDevice('t5_sensor')
rt1_sensor = robot.getDevice('rt1_sensor'); rt2_sensor = robot.getDevice('rt2_sensor'); rt3_sensor = robot.getDevice('rt3_sensor'); rt4_sensor = robot.getDevice('rt4_sensor'); rt5_sensor = robot.getDevice('rt5_sensor')

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

# 動作狀態機
special_mode = None  # None, 'a_left', 'a_right', 'a_return'
prev_a_pressed = False

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
    """左腳：t3=-60, t4=30, t5=-30（保留 t1,t2）"""
    global left_target_deg
    current_left = [round(math.degrees(s.getValue()), 1) for s in left_sensors]
    left_target_deg = [current_left[0], current_left[1], -60, 30, 10]
    print(f"=== 按 a: 左腳 t3=-60°, t4=30°, t5=-30° (t1={current_left[0]}, t2={current_left[1]}) ===")

def special_a_right_pose():
    """右腳：rt3=-60, rt4=30, rt5=-30（保留 rt1,rt2）"""
    global right_target_deg
    current_right = [round(math.degrees(s.getValue()), 1) for s in right_sensors]
    right_target_deg = [current_right[0], current_right[1], 0, 0, 0]
    print(f"=== 左腳完成 → 右腳 t3=-60°, t4=30°, t5=-30° (rt1={current_right[0]}, rt2={current_right[1]}) ===")

# ========================
# 單關節控制
# ========================

def adjust_joint(side, idx, delta):
    if side == 'left':
        left_target_deg[idx] += delta
    else:
        right_target_deg[idx] += delta

def handle_key_adjust(key):
    if key == ord('1'): adjust_joint('left', 0, 10)
    elif key == ord('2'): adjust_joint('left', 1, 10)
    elif key == ord('3'): adjust_joint('left', 2, 10)
    elif key == ord('4'): adjust_joint('left', 3, 10)
    elif key == ord('5'): adjust_joint('left', 4, 10)
    elif key in [ord('Q'),ord('q')]: adjust_joint('left', 0, -10)
    elif key in [ord('W'),ord('w')]: adjust_joint('left', 1, -10)
    elif key in [ord('E'),ord('e')]: adjust_joint('left', 2, -10)
    elif key in [ord('R'),ord('r')]: adjust_joint('left', 3, -10)
    elif key in [ord('T'),ord('t')]: adjust_joint('left', 4, -10)
    elif key == ord('6'): adjust_joint('right', 0, 10)
    elif key == ord('7'): adjust_joint('right', 1, 10)
    elif key == ord('8'): adjust_joint('right', 2, 10)
    elif key == ord('9'): adjust_joint('right', 3, 10)
    elif key == ord('0'): adjust_joint('right', 4, 10)
    elif key in [ord('Y'),ord('y')]: adjust_joint('right', 0, -10)
    elif key in [ord('U'),ord('u')]: adjust_joint('right', 1, -10)
    elif key in [ord('I'),ord('i')]: adjust_joint('right', 2, -10)
    elif key in [ord('O'),ord('o')]: adjust_joint('right', 3, -10)
    elif key in [ord('P'),ord('p')]: adjust_joint('right', 4, -10)

# ========================
# 初始化
# ========================

robot.step(timestep)
stand_pose()
set_motor_targets()

print("\n" + "="*70)
print("       雙足機器人控制面板（每按一次 a → 左腳 → 右腳 → 歸零）")
print("="*70)
print("s : 站立 (全部歸零)")
print("k : 蹲姿")
print("a : 左腳(t3,t4,t5) → 右腳(rt3,rt4,rt5) → 強制歸零")
print("1-5 / q-w-e-r-t : 左腿 ±10°")
print("6-0 / y-u-i-o-p : 右腿 ±10°")
print("="*70)

# ========================
# 主迴圈
# ========================

moving = False

while robot.step(timestep) != -1:
    key = keyboard.getKey()
    a_pressed = (key == ord('A') or key == ord('a'))

    # === 上升沿偵測：按下 a 的瞬間 ===
    if a_pressed and not prev_a_pressed and special_mode is None:
        print("檢測到 a 按下 → 啟動完整序列：左腳 → 右腳 → 歸零")
        special_a_left_pose()
        set_motor_targets(FAST_MOVE_TIME)
        moving = True
        special_mode = 'a_left'

    prev_a_pressed = a_pressed

    # === 其他按鍵（可中斷 a 序列）===
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

    # === 動作完成偵測 ===
    if moving and is_arrived():
        stop_all_motors()
        moving = False
        print("階段完成")

        if special_mode == 'a_left':
            print("左腳動作完成 → 啟動右腳")
            special_a_right_pose()
            set_motor_targets(FAST_MOVE_TIME)
            moving = True
            special_mode = 'a_right'

        elif special_mode == 'a_right':
            print("右腳動作完成 → 強制歸零")
            stand_pose()
            set_motor_targets(MOVE_TIME)
            moving = True
            special_mode = 'a_return'

        elif special_mode == 'a_return':
            print("完整序列結束！所有關節已歸零，可再次按 a")
            special_mode = None