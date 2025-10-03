import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Webots 機構與坐標系參數 ---
# 全局平移量：左馬達 J_L 在 Webots 世界坐標系中的位置 (X, Y)
GLOBAL_X_OFFSET = -0.2
GLOBAL_Y_OFFSET = -0.25

# 五連桿機構參數
d = 0.4    # 基座距離 L5 (J_R 局部 X 座標)
a = 0.32   # 旋轉臂長 L1, L4
b = 0.32   # 耦合連桿長 L2, L3

# 繪圖路徑參數 (局部坐標系下的正方形)
S = 0.4      # 正方形邊長
Y_offset_local = 0.06 # 底部水平線 Y 座標 (相對於 J_L-J_R 連線)

# --- 1. 逆運動學 (IK) 核心函式 ---
# IK 必須以局部坐標 (J_L 為原點) 進行計算
def inverse_kinematics_5bar(x_local, y_local, d, a, b, left_elbow_up, right_elbow_up):
    """
    x_local, y_local 是相對於左馬達 (0, 0) 的局部座標。
    """
    
    # 1. 求解 t1 (左臂, J_L 為原點)
    try:
        cos_t_prime = (x_local**2 + y_local**2 - a**2 - b**2) / (2 * a * b)
        if abs(cos_t_prime) > 1.0001: return np.nan, np.nan # 不可達
            
        cos_t_prime = np.clip(cos_t_prime, -1.0, 1.0)
        t_prime_sol = np.arccos(cos_t_prime)
        t_prime = t_prime_sol if left_elbow_up else -t_prime_sol
        
        K1 = a + b * np.cos(t_prime)
        K2 = b * np.sin(t_prime)
        t1 = np.arctan2(y_local, x_local) - np.arctan2(K2, K1)
        t1 = np.arctan2(np.sin(t1), np.cos(t1))
    except:
        t1 = np.nan
        
    # 2. 求解 t2 (右臂, J_R 為原點)
    x_prime = x_local - d
    y_prime = y_local
    try:
        cos_t_double_prime = (x_prime**2 + y_prime**2 - a**2 - b**2) / (2 * a * b)
        if abs(cos_t_double_prime) > 1.0001: return np.nan, np.nan # 不可達
            
        cos_t_double_prime = np.clip(cos_t_double_prime, -1.0, 1.0)
        t_double_prime_sol = np.arccos(cos_t_double_prime)
        t_double_prime = t_double_prime_sol if right_elbow_up else -t_double_prime_sol
        
        K3 = a + b * np.cos(t_double_prime)
        K4 = b * np.sin(t_double_prime)
        t2 = np.arctan2(y_prime, x_prime) - np.arctan2(K4, K3)
        t2 = np.arctan2(np.sin(t2), np.cos(t2))
    except:
        t2 = np.nan

    if np.isnan(t1) or np.isnan(t2):
        return None, None
    
    return t1, t2


# --- 2. 路徑生成與全局平移 ---
num_points_per_side = 50

# 局部坐標頂點: P1=(0.0, 0.06), P2=(0.4, 0.06), P3=(0.4, 0.46), P4=(0.0, 0.46)
local_points_x = np.array([0.0, d, d, 0.0, 0.0])
local_points_y = np.array([Y_offset_local, Y_offset_local, Y_offset_local + S, Y_offset_local + S, Y_offset_local])

local_path_x, local_path_y = [], []
for i in range(4):
    x_segment = np.linspace(local_points_x[i], local_points_x[i+1], num_points_per_side, endpoint=False)
    y_segment = np.linspace(local_points_y[i], local_points_y[i+1], num_points_per_side, endpoint=False)
    local_path_x.extend(x_segment)
    local_path_y.extend(y_segment)

local_path_x.append(local_points_x[-1])
local_path_y.append(local_points_y[-1])
num_points = len(local_path_x)

# 應用全局平移，得到 Webots 世界座標 (用於繪圖)
global_path_x = np.array(local_path_x) + GLOBAL_X_OFFSET
global_path_y = np.array(local_path_y) + GLOBAL_Y_OFFSET


# --- 3. 計算四種 IK 解的路徑 ---
solutions = {
    'S1 (左上, 右上)': (True, True, []),  
    'S2 (左上, 右下)': (True, False, []), 
    'S3 (左下, 右上)': (False, True, []), 
    'S4 (左下, 右下)': (False, False, []),
}

# IK 運算使用局部坐標
for x_local, y_local in zip(local_path_x, local_path_y):
    for key in solutions:
        left_up, right_up, path_list = solutions[key]
        t1, t2 = inverse_kinematics_5bar(x_local, y_local, d, a, b, left_up, right_up)
        path_list.append((t1, t2))

# --- 4. Matplotlib 動態模擬設定 ---
fig = plt.figure(figsize=(12, 12))
axs = []
solution_keys = list(solutions.keys())

# 全局坐標系的基座位置
J_L_GLOBAL = (0 + GLOBAL_X_OFFSET, 0 + GLOBAL_Y_OFFSET)
J_R_GLOBAL = (d + GLOBAL_X_OFFSET, 0 + GLOBAL_Y_OFFSET)

# 設置繪圖範圍以包含 Webots 的全局位置
X_MIN = GLOBAL_X_OFFSET - 0.1
X_MAX = GLOBAL_X_OFFSET + d + 0.1
Y_MIN = GLOBAL_Y_OFFSET - 0.1
Y_MAX = GLOBAL_Y_OFFSET + a + b + 0.1 

for i in range(4):
    ax = fig.add_subplot(2, 2, i + 1, aspect='equal', xlim=(X_MIN, X_MAX), ylim=(Y_MIN, Y_MAX))
    ax.set_title(f'{solution_keys[i]}')
    ax.set_xlabel('Global X Position (m)')
    ax.set_ylabel('Global Y Position (m)')
    ax.grid(True)
    
    # 繪製基座 (全局坐標)
    ax.plot([J_L_GLOBAL[0], J_R_GLOBAL[0]], [J_L_GLOBAL[1], J_R_GLOBAL[1]], 'k-', linewidth=1)
    ax.plot(J_L_GLOBAL[0], J_L_GLOBAL[1], 'ko', markersize=5, label='J_L Global')
    ax.plot(J_R_GLOBAL[0], J_R_GLOBAL[1], 'ko', markersize=5, label='J_R Global')
    
    # 繪製目標路徑 (全局坐標)
    ax.plot(global_path_x, global_path_y, 'k--', alpha=0.3, label='Target Path')
    axs.append(ax)

# 動態繪圖元素
lines_L1 = []; lines_L4 = []; lines_L2 = []; lines_L3 = []
dots_P = []; trace_P = []

for ax in axs:
    lines_L1.append(ax.plot([], [], 'r-', linewidth=3, label='$L_1/L_4$')[0])
    lines_L4.append(ax.plot([], [], 'r-', linewidth=3)[0])
    lines_L2.append(ax.plot([], [], 'b--', linewidth=2, label='$L_2/L_3$')[0])
    lines_L3.append(ax.plot([], [], 'b--', linewidth=2)[0])
    dots_P.append(ax.plot([], [], 'go', markersize=8, label='End-effector P')[0])
    trace_P.append(ax.plot([], [], 'g:', alpha=0.6)[0])

# 初始化畫布
def init():
    all_elements = []
    for i in range(4):
        lines_L1[i].set_data([], []); lines_L4[i].set_data([], [])
        lines_L2[i].set_data([], []); lines_L3[i].set_data([], [])
        dots_P[i].set_data([], []); trace_P[i].set_data([], [])
        all_elements.extend([lines_L1[i], lines_L4[i], lines_L2[i], lines_L3[i], dots_P[i], trace_P[i]])
    return all_elements

# 更新動畫
def update(frame):
    all_elements = []
    
    for i in range(4):
        path_list = solutions[solution_keys[i]][2]
        t1, t2 = path_list[frame]

        # 繪圖軌跡使用全局坐標
        current_x_global = global_path_x[:frame+1]
        current_y_global = global_path_y[:frame+1]
        
        if np.isnan(t1):
            # 點不可達，隱藏連桿
            lines_L1[i].set_data([], []); lines_L4[i].set_data([], [])
            lines_L2[i].set_data([], []); lines_L3[i].set_data([], [])
            dots_P[i].set_data([], [])
        else:
            # 計算關節局部座標 P_L, P_R (以 J_L 為原點)
            x_L_local = a * np.cos(t1)
            y_L_local = a * np.sin(t1)
            x_R_local = d + a * np.cos(t2)
            y_R_local = a * np.sin(t2)
            
            # 轉換為全局座標 (用於繪圖)
            x_L_global = x_L_local + GLOBAL_X_OFFSET
            y_L_global = y_L_local + GLOBAL_Y_OFFSET
            x_R_global = x_R_local + GLOBAL_X_OFFSET
            y_R_global = y_R_local + GLOBAL_Y_OFFSET
            x_P_global, y_P_global = global_path_x[frame], global_path_y[frame] 

            # 更新連桿數據 (全局坐標)
            lines_L1[i].set_data([J_L_GLOBAL[0], x_L_global], [J_L_GLOBAL[1], y_L_global])
            lines_L4[i].set_data([J_R_GLOBAL[0], x_R_global], [J_R_GLOBAL[1], y_R_global])
            lines_L2[i].set_data([x_L_global, x_P_global], [y_L_global, y_P_global])
            lines_L3[i].set_data([x_R_global, x_P_global], [y_R_global, y_P_global])
            
            # 更新末端執行器 P (全局坐標)
            dots_P[i].set_data([x_P_global], [y_P_global])
            
            # 更新軌跡
            trace_P[i].set_data(current_x_global, current_y_global)

        all_elements.extend([lines_L1[i], lines_L4[i], lines_L2[i], lines_L3[i], dots_P[i], trace_P[i]])

    return all_elements

# 建立動畫
ani = FuncAnimation(fig, update, frames=num_points, init_func=init, blit=True, interval=50)

plt.suptitle(f'Five-Bar Plotter Dynamic IK Solutions (Global Webots Coords)', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.96])

plt.show()

print("\n--- 模擬總結 ---")
print(f"機構參數: d={d}m, a=b={a}m (四連桿長度相等)")
print(f"Webots 全局平移: X={GLOBAL_X_OFFSET}m, Y={GLOBAL_Y_OFFSET}m")
print(f"四種 IK 解的動態模擬已在全局座標系中展示。")