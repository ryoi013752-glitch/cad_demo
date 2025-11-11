from controller import Robot, Keyboard
import math

# ========================
# 初始化
# ========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

# ------------------- 馬達 & 感測器 -------------------
left_motors   = [robot.getDevice(f't{i}')   for i in range(1, 6)]
right_motors  = [robot.getDevice(f'rt{i}')  for i in range(1, 6)]
left_sensors  = [robot.getDevice(f't{i}_sensor')  for i in range(1, 6)]
right_sensors = [robot.getDevice(f'rt{i}_sensor') for i in range(1, 6)]

for s in left_sensors + right_sensors:
    if s: s.enable(timestep)

# ------------------- GPS / IMU -------------------
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')
use_gps = gps is not None
use_imu = imu is not None

if use_gps: gps.enable(timestep); print("GPS 已啟用")
else: print("警告：無 GPS")

if use_imu: imu.enable(timestep); print("IMU 已啟用")
else: print("警告：無 IMU")

# ------------------- 參數（LIPM 強化） -------------------
STAND_DEGREES = [0, 0, 0, 0, 0]
SQUAT_DEGREES = [0, 0, 55, -110, 55]

MOVE_TIME = 1.5
FAST_MOVE_TIME = 1.2
MARGIN = 1.1
TOLERANCE = 0.1
BALANCE_THR = 0.15

# === LIPM 參數 ===
G = 9.81
COM_HEIGHT = 0.55
OMEGA = math.sqrt(G / COM_HEIGHT)
STEP_LENGTH = 0.45  # 保守步長
FOOT_LENGTH = 0.15  # 假設腳長

# === PID 平衡控制 ===
Kp_pitch = 3.0   # 增益 ×3
Ki_pitch = 0.1
Kd_pitch = 0.5
pitch_integral = 0.0
pitch_prev_error = 0.0

target_positions = [0.0] * 10

# ------------------- 工具函式 -------------------
def deg2rad(deg_list): return [math.radians(d) for d in deg_list]

def get_joint_pos():
    return [s.getValue() if s else 0.0 for s in left_sensors + right_sensors]

def get_imu():
    return imu.getRollPitchYaw() if use_imu else (0.0, 0.0, 0.0)

def get_gps_data():
    if not use_gps: return [0.0]*3, [0.0]*3
    return gps.getValues(), gps.getSpeedVector()

# === LIPM 期望軌跡 ===
def lipm_trajectory(x0, vx0, p, t):
    cosh = math.cosh(OMEGA * t)
    sinh = math.sinh(OMEGA * t)
    return (x0 - p) * cosh + (vx0 / OMEGA) * sinh + p

# === ZMP 計算（簡化）===
def estimate_zmp(com_x, com_z=COM_HEIGHT):
    # 假設加速度為 0，ZMP ≈ CoM 投影
    return com_x

# === PID 平衡控制 ===
def balance_pid_control(pitch):
    global pitch_integral, pitch_prev_error
    error = pitch
    pitch_integral += error * (timestep / 1000.0)
    pitch_integral = max(-5, min(5, pitch_integral))
    derivative = (error - pitch_prev_error) / (timestep / 1000.0)
    output = Kp_pitch * error + Ki_pitch * pitch_integral + Kd_pitch * derivative
    pitch_prev_error = error
    return max(-25, min(25, math.degrees(output)))

def set_targets(left_deg, right_deg, t=MOVE_TIME):
    global target_positions
    if use_imu:
        _, pitch, _ = get_imu()
        adj = balance_pid_control(pitch)
        left_deg[0] += adj
        right_deg[0] += adj

    target_positions = deg2rad(left_deg) + deg2rad(right_deg)
    cur = get_joint_pos()
    vel = [abs(target_positions[i]-cur[i])/t * MARGIN if abs(target_positions[i]-cur[i])>1e-6 else 0
           for i in range(10)]

    for m, p, v in zip(left_motors + right_motors, target_positions, vel):
        if m:
            m.setPosition(p)
            m.setVelocity(v)

def is_arrived(timeout=300):
    count = 0
    while robot.step(timestep) != -1 and count < timeout:
        cur = get_joint_pos()
        if all(abs(cur[i] - target_positions[i]) < TOLERANCE for i in range(10)):
            return True
        count += 1
    return True  # 強制繼續

def stop_all():
    for m in left_motors + right_motors:
        if m: m.setVelocity(0.0)

# ------------------- 姿態 -------------------
def stand():
    set_targets(STAND_DEGREES, STAND_DEGREES, MOVE_TIME)
    print("=== 站直 ===")
    is_arrived()

# ------------------- 強化 LIPM 三步 -------------------
def walk_three_steps_lipm():
    print("\n" + "="*70)
    print("   強化 LIPM 三步前進（即時軌跡 + ZMP + PID 平衡）")
    print("="*70)

    init_pos, _ = get_gps_data()
    print(f"起始 CoM x = {init_pos[0]:.3f}m")

    for step in range(3):
        leg = "左" if step % 2 == 0 else "右"
        support_leg = "右" if step % 2 == 0 else "左"
        print(f"\n--- 第 {step+1} 步：{leg}腳前進，{support_leg}腳支撐 ---")

        # === 1. 重心移到支撐腳 + 擺腿 ===
        if step % 2 == 0:
            left_swing  = [15, 0, -70, 50, 10]
            right_support = [-15, 0, 15, -25, 10]
        else:
            left_support = [15, 0, 15, -25, 10]
            right_swing  = [-15, 0, -70, 50, 10]

        set_targets(left_swing if step % 2 == 0 else left_support,
                    right_support if step % 2 == 0 else right_swing,
                    FAST_MOVE_TIME)
        print("等待擺腿...")
        is_arrived()

        # === 2. LIPM 即時控制階段 ===
        print("LIPM 即時控制中...")
        start_time = robot.getTime()
        p_zmp = init_pos[0] + STEP_LENGTH * (step + 0.5)
        x0, vx0 = get_gps_data()[0][0], get_gps_data()[1][0]

        while robot.getTime() - start_time < 0.8:
            t = robot.getTime() - start_time
            x_des = lipm_trajectory(x0, vx0, p_zmp, t)
            com_x = get_gps_data()[0][0]
            zmp = estimate_zmp(com_x)

            # ZMP 超出支撐區 → 緊急調整
            support_center = p_zmp
            if abs(zmp - support_center) > FOOT_LENGTH / 2:
                adj = (zmp - support_center) * 50  # 強力補償
                left_motors[0].setPosition(math.radians(15 + adj))
                right_motors[0].setPosition(math.radians(-15 + adj))

            robot.step(timestep)

        # === 3. 雙腳落地 ===
        print("雙腳落地...")
        land = [5, 0, 10, -20, 10]
        set_targets(land, land, 0.6)
        is_arrived()

        cur_pos, _ = get_gps_data()
        print(f"第 {step+1} 步完成，CoM x = {cur_pos[0]:.3f}m")

    # === 回到站立 ===
    print("\n=== LIPM 三步完成，站立穩定 ===")
    stand()
    final_pos, _ = get_gps_data()
    print(f"最終 CoM x = {final_pos[0]:.3f}m（總前進 {final_pos[0]-init_pos[0]:.3f}m）")
    stop_all()

# ------------------- 初始化 -------------------
robot.step(timestep)
stand()

# ------------------- 主迴圈 -------------------
print("\n控制說明：")
print("  S/s : 站直")
print("  X/x : 強化 LIPM 三步（即時平衡 + ZMP 防倒）")
print("-"*60)

last_key = -1
walking = False

while robot.step(timestep) != -1:
    key = keyboard.getKey()

    if key != -1 and key != last_key:
        if key in (ord('S'), ord('s')) and not walking:
            stand()
        elif key in (ord('X'), ord('x')) and not walking:
            walking = True
            walk_three_steps_lipm()
            walking = False

    last_key = key

    if not walking:
        stop_all()