import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sympy as sp

# --- 符號與逆運動學定義 ---
x, y = sp.symbols('x y', real=True)
t1, t2, t3, t4 = sp.symbols('t1 t2 t3 t4', real=True)

# 連桿長度
L1 = 22  # AB
L2 = 28  # BC
L3 = 28  # CD
L4 = 22  # DE
B = 20   # 馬達間距

# 逆運動學方程式
# 使用簡單的三角學推導（具體推導過程根據圖示進行）
eq1 = sp.Eq((L1 * sp.cos(t1) - x)**2 + (L1 * sp.sin(t1) - y)**2, L2**2)
eq2 = sp.Eq((B - L1 * sp.cos(t2) - x)**2 + (L1 * sp.sin(t2) - y)**2, L2**2)

# 解出所有逆運動學解
solutions = sp.solve([eq1, eq2], (t1, t2), dict=True)

# 轉為 lambda 函數列表
ik_solutions = []
for i, sol in enumerate(solutions):
    t1_expr = sol[t1]
    t2_expr = sol[t2]
    t1_func = sp.lambdify((x, y), t1_expr, 'numpy')
    t2_func = sp.lambdify((x, y), t2_expr, 'numpy')
    ik_solutions.append((t1_func, t2_func))

# 四個目標點
target_points = [(0, 40), (20, 40), (20, 20), (0, 20)]

# 初始化圖形
fig, ax = plt.subplots()
ax.set_xlim(-L1 - L2, L1 + L2)
ax.set_ylim(-L1 - L2, L1 + L2)
ax.set_aspect('equal')
line1, = ax.plot([], [], 'bo-', lw=2)
line2, = ax.plot([], [], 'ro-', lw=2)

# 用於繪製通過四個目標點的直線
path_line, = ax.plot([], [], 'g--', lw=1, alpha=0.6)

# 初始化臂的狀態
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    path_line.set_data([], [])
    return line1, line2, path_line,

# 更新函數：每個目標點的動態模擬
def update(frame):
    x_target, y_target = target_points[frame]

    # 從每個解中計算角度
    t1_vals = [t1_func(x_target, y_target) for t1_func, _ in ik_solutions]
    t2_vals = [t2_func(x_target, y_target) for _, t2_func in ik_solutions]

    # 確保選擇合理的解（例如，選擇最小角度差的解）
    best_solution = None
    min_distance = float('inf')
    
    for t1_val, t2_val in zip(t1_vals, t2_vals):
        # 計算角度差的距離（簡單選擇與先前解最接近的角度）
        distance = np.abs(t1_val - t2_val)
        
        if distance < min_distance:
            min_distance = distance
            best_solution = (t1_val, t2_val)

    # 使用選中的最佳解
    t1, t2 = best_solution
    
    # 計算臂端座標
    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)
    
    x2 = x1 + L2 * np.cos(t2)
    y2 = y1 + L2 * np.sin(t2)
    
    # 更新圖形數據
    line1.set_data([0, x1], [0, y1])  # 第1根臂的位置
    line2.set_data([x1, x2], [y1, y2])  # 第2根臂的位置
    
    # 繪製目標點之間的連線
    path_x = [pt[0] for pt in target_points] + [target_points[0][0]]  # 添加首尾連接
    path_y = [pt[1] for pt in target_points] + [target_points[0][1]]  # 添加首尾連接
    path_line.set_data(path_x, path_y)  # 更新目標點之間的直線

    return line1, line2, path_line,

# 創建動畫
ani = FuncAnimation(fig, update, frames=range(len(target_points)),
                    init_func=init, blit=True, interval=1000)

plt.show()
