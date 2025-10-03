import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cmath
import math

# 馬達中心點
A = (0, 0)       # 左馬達
B = (400, 0)     # 右馬達

L = 320  # 連桿長度

# --- Solution 4 ---
def t1_sol4(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))

def t2_sol4(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

# 產生路徑 (四條邊分別100點，共400點)
def generate_path():
    n = 100
    bottom = [(x, 30) for x in np.linspace(0, 400, n)]
    right = [(400, y) for y in np.linspace(30, 430, n)]
    top = [(x, 430) for x in np.linspace(400, 0, n)]
    left = [(0, y) for y in np.linspace(430, 30, n)]
    return bottom, right, top, left

bottom, right, top, left = generate_path()
path = bottom + right + top + left

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-100, 500)
ax.set_ylim(0, 500)
ax.set_aspect('equal')
ax.grid(True)

line, = ax.plot([], [], 'o-', lw=3, color='blue')
pen, = ax.plot([], [], 'ro', markersize=6)

def normalize_angle_deg(angle):
    """標準化角度到 [0, 360)"""
    angle = angle % 360
    return angle

def angle_relative_to_left_motor(theta1):
    """t1 以左馬達水平線向右為0度，逆時針為正"""
    deg = math.degrees(theta1.real)
    return normalize_angle_deg(deg)

def angle_relative_to_right_motor(theta2):
    """
    t2 原本是以右馬達水平線向左為0度，順時針為正，
    轉成以右馬達水平線向右為0度，逆時針為正:
    t2_new = (540 - t2_old) mod 360
    """
    deg_old = math.degrees(theta2.real)
    deg_new = (540 - deg_old) % 360
    return deg_new

def get_5_indices(segment_length, offset):
    return [int(offset + i * (segment_length-1) / 4) for i in range(5)]

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

segment_names = ['bottom', 'right', 'top', 'left']

printed = {name: set() for name in segment_names}

def init():
    line.set_data([], [])
    pen.set_data([], [])
    return line, pen

def animate(i):
    x, y = path[i]

    theta1 = t1_sol4(x, y)
    theta2 = t2_sol4(x, y)

    C = (L * cmath.cos(theta1).real, L * cmath.sin(theta1).real)
    D = (B[0] - L * cmath.cos(theta2).real, L * cmath.sin(theta2).real)

    xs = [A[0], C[0], x, D[0], B[0]]
    ys = [A[1], C[1], y, D[1], B[1]]

    line.set_data(xs, ys)
    pen.set_data([x], [y])

    for name in segment_names:
        if i in indices_to_print[name] and i not in printed[name]:
            t1_deg = angle_relative_to_left_motor(theta1)
            t2_deg = angle_relative_to_right_motor(theta2)
            print(f"[{name.capitalize()}] index={i}, Point=({x:.1f},{y:.1f}), t1={t1_deg:.4f}°, t2={t2_deg:.4f}°")
            printed[name].add(i)

    return line, pen

ani = animation.FuncAnimation(fig, animate, frames=len(path),
                              init_func=init, interval=30, blit=True)

plt.title("五連桿機構動畫，連桿長度320，通過指定四點")
plt.show()
