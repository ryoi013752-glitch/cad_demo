import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cmath

# 馬達中心點
A = (0, 0)       # 左馬達
B = (400, 0)     # 右馬達

L = 320  # 連桿長度

# --- Solution 4 ---
def t1_sol3(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol3(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

# 注意！這裡的常數跟你給的連桿長度320、馬達位置400有關，原公式中640 = 2*320，
# 39936000000 = (320^2)^2 * 1e4 相關，需要根據實際 L 和 B 調整公式（建議用 sympy重新推導）

# 如果暫時不重新推導公式，先直接用數字生成路徑：

def generate_path():
    # 四個點分別為 (0,30), (400,30), (400,430), (0,430)
    # 路徑順序：下方左->右、右邊下->上、上方右->左、左邊上->下
    n = 100
    bottom = [(x, 30) for x in np.linspace(0, 400, n)]
    right = [(400, y) for y in np.linspace(30, 430, n)]
    top = [(x, 430) for x in np.linspace(400, 0, n)]
    left = [(0, y) for y in np.linspace(430, 30, n)]
    return bottom + right + top + left

path = generate_path()

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-100, 500)
ax.set_ylim(0, 500)
ax.set_aspect('equal')
ax.grid(True)

line, = ax.plot([], [], 'o-', lw=3, color='blue')
pen, = ax.plot([], [], 'ro', markersize=6)

def init():
    line.set_data([], [])
    pen.set_data([], [])
    return line, pen

def animate(i):
    x, y = path[i]

    # 計算角度
    theta1 = t1_sol3(x, y)
    theta2 = t2_sol3(x, y)

    # 連桿點計算 (用L=320)
    C = (L * cmath.cos(theta1).real, L * cmath.sin(theta1).real)  # 左連桿端點
    D = (B[0] - L * cmath.cos(theta2).real, L * cmath.sin(theta2).real)  # 右連桿端點

    xs = [A[0], C[0], x, D[0], B[0]]
    ys = [A[1], C[1], y, D[1], B[1]]

    line.set_data(xs, ys)
    pen.set_data([x], [y])
    return line, pen

ani = animation.FuncAnimation(fig, animate, frames=len(path),
                              init_func=init, interval=30, blit=True)

plt.title("五連桿機構動畫，連桿長度320，通過指定四點")
plt.show()
