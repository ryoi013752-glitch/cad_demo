from controller import Robot
import math

# --- 機構參數 ---
L_VAL = 0.32
AX, AY = -0.2, -0.25
EX, EY = 0.2, -0.25

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

t1 = robot.getDevice("t1")
t2 = robot.getDevice("t2")
sensor1 = robot.getDevice("sensor1")
sensor2 = robot.getDevice("sensor2")

sensor1.enable(TIME_STEP)
sensor2.enable(TIME_STEP)

t1.setVelocity(5.0)
t2.setVelocity(5.0)

# 等感測器穩定
for _ in range(10):
    robot.step(TIME_STEP)

angle1_rad = sensor1.getValue()
angle2_rad = sensor2.getValue()

# 計算 B 點位置（馬達1連桿末端）
Bx = AX + L_VAL * math.cos(angle1_rad)
By = AY + L_VAL * math.sin(angle1_rad)

# 計算 D 點位置（馬達2連桿末端）
Dx = EX + L_VAL * math.cos(angle2_rad)
Dy = EY + L_VAL * math.sin(angle2_rad)

# 計算 B 與 D 角度相對水平線（以角度表示）
# 因為向右為水平線0度，逆時針為正
angle_B_deg = math.degrees(math.atan2(By - AY, Bx - AX))
angle_D_deg = math.degrees(math.atan2(Dy - EY, Dx - EX))

print(f"馬達 t1 對水平線角度: {angle_B_deg:.2f} 度")
print(f"馬達 t2 對水平線角度: {angle_D_deg:.2f} 度")

while robot.step(TIME_STEP) != -1:
    pass
