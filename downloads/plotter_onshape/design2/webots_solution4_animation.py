import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cmath
import math
from scipy.optimize import fsolve

# 馬達中心點，單位為公尺
A = (-0.2, -0.25)       # 左馬達
B = (0.2, -0.25)        # 右馬達 (400mm = 0.4m，除以2等於0.2)

# 連桿長度，單位公尺 (320mm = 0.32m)
L = 0.32

def solve_angles(x, y, A, B, L):
    # 左馬達角度方程組
    def equations_left(t):
        theta1 = t[0]
        return [A[0] + L * np.cos(theta1) - x,
                A[1] + L * np.sin(theta1) - y]

    # 右馬達角度方程組
    def equations_right(t):
        theta2 = t[0]
        return [B[0] - L * np.cos(theta2) - x,
                B[1] + L * np.sin(theta2) - y]

    # 初始猜測：從馬達位置指向點的角度
    t1_guess = math.atan2(y - A[1], x - A[0])
    t2_guess = math.atan2(y - B[1], B[0] - x)

    t1_solution = fsolve(equations_left, [t1_guess])[0]
    t2_solution = fsolve(equations_right, [t2_guess])[0]

    return t1_solution, t2_solution

def angle_deg_normalize(angle):
    """將角度標準化到 [0, 360) 度"""
    return angle % 360

def angle_relative_to_left_motor(theta1):
    """t1 以左馬達水平線向右為0度，逆時針為正"""
    deg = math.degrees(theta1)
    return angle_deg_normalize(deg)

def angle_relative_to_right_motor(theta2):
    """
    t2 原本是以右馬達水平線向左為0度，順時針為正，
    轉成以右馬達水平線向右為0度，逆時針為正:
    t2_new = (540 - t2_old) mod 360
    """
    deg_old = math.degrees(theta2)
    deg_new = (540 - deg_old) % 360
    return deg_new

# 路徑座標單位從 mm 轉成 m
def generate_path():
    n = 100
    bottom = [(x / 1000, 30 / 1000) for x in np.linspace(0, 400, n)]   # y=30mm
    right = [(400 / 1000, y / 1000) for y in np.linspace(30, 430, n)]
    top = [(x / 1000, 430 / 1000) for x in np.linspace(400, 0, n)]
    left = [(0 / 1000, y / 1000) for y in np.linspace(430, 30, n)]
    return bottom, right, top, left

bottom, right, top, left = generate_path()
path = bottom + right + top + left

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-0.3, 0.6)
ax.set_ylim(-0.3, 0.6)
ax.set_aspect('equal')
ax.grid(True)

line, = ax.plot([], [], 'o-', lw=3, color='blue')
pen, = ax.plot([], [], 'ro', markersize=6)

segment_names = ['bottom', 'right', 'top', 'left']

def get_5_indices(segment_length, offset):
    return [int(offset + i * (segment_length - 1) / 4) for i in range(5)]

bottom_indices = get_5_indices(len(bottom), 0)
right_indices = get_5_indices(len(right), len(bottom))
top_indices = get_5_indices(len(top), len(bottom) + len(right))
left_indices = get_5_indices(len(left), len(bottom) + len(right) + len(top))

indices_to_print = {
    'bottom': bottom_indices,
    'right': right_indices,
    'top': top_indices,
    'left': left_indices,
}

printed = {name: set() for name in segment_names}

def init():
    line.set_data([], [])
    pen.set_data([], [])
    return line, pen

def animate(i):
    x, y = path[i]
    try:
        t1, t2 = solve_angles(x, y, A, B, L)
    except Exception as e:
        print(f"Error solving angles at index {i}, point ({x:.4f},{y:.4f}): {e}")
        return line, pen

    # 計算連桿末端點位置
    C = (A[0] + L * np.cos(t1), A[1] + L * np.sin(t1))   # 左連桿端點
    D = (B[0] - L * np.cos(t2), B[1] + L * np.sin(t2))   # 右連桿端點

    xs = [A[0], C[0], x, D[0], B[0]]
    ys = [A[1], C[1], y, D[1], B[1]]

    line.set_data(xs, ys)
    pen.set_data([x], [y])

    for name in segment_names:
        if i in indices_to_print[name] and i not in printed[name]:
            t1_deg = angle_relative_to_left_motor(t1)
            t2_deg = angle_relative_to_right_motor(t2)
            print(f"[{name.capitalize()}] index={i}, Point=({x*1000:.1f}mm,{y*1000:.1f}mm), t1={t1_deg:.4f}°, t2={t2_deg:.4f}°")
            printed[name].add(i)

    return line, pen

ani = animation.FuncAnimation(fig, animate, frames=len(path),
                              init_func=init, interval=30, blit=True)

plt.title("五連桿機構動畫，連桿長度0.32m，馬達位置(-0.2, -0.25)m，單位為公尺")
plt.show()
