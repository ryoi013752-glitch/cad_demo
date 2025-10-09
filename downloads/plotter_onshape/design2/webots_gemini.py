import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sympy as sp
import cmath

# --- 1. 參數設定與逆運動學求解 (確保 ik_solutions 被定義) ---
# 符號定義
x, y = sp.symbols('x y', real=True)
t1, t2 = sp.symbols('t1 t2', real=True)

# 參數設定 (單位: m)
L_VAL = 0.32  # 連桿長度 L = 0.32m
B_VAL = 0.4   # 馬達間距 B = 0.4 m
# 左側馬達中心點 A 位於 (-0.2, -0.25)
ax = -0.2
ay = -0.25
# 右側馬達中心點 E 位於 (0.2, -0.25)
ex = 0.2
ey = -0.25
# link1 從右側水平線作為角度 0, 逆時針方向為正 (t1)
# B 點座標為 (ax+L_VAL*sp.cos(t1), ay+L_VAL*sp.sin(t1))
bx = ax + L_VAL*sp.cos(t1)
by = ay + L_VAL*sp.sin(t1)
# link2 從左側水平線作為角度 0, 順時針方向為正 (t2)
# D 點座標為 (ex+L_VAL*sp.cos(t2), ey+L_VAL*sp.sin(t2))
dx = ex + L_VAL*sp.cos(t2)
dy = ey + L_VAL*sp.sin(t2)

# 逆運動學方程式 (P 點 (x, y) 到 B 和 D 點的距離為 L_VAL)
eq1 = sp.Eq((bx - x)**2 + (by - y)**2, L_VAL**2)
eq2 = sp.Eq((dx - x)**2 + (dy - y)**2 , L_VAL**2)

# 解出所有逆運動學解
solutions = sp.solve([eq1, eq2], (t1, t2), dict=True)

# 將 SymPy 解轉換為 NumPy/cMath 函數，並存儲在 ik_solutions 中
ik_solutions = []
for i, sol in enumerate(solutions):
    t1_expr = sol[t1]
    t2_expr = sol[t2]
    # 使用 cmath 進行 lambdify (用於模擬，處理複數)
    t1_func = sp.lambdify((x, y), t1_expr, 'numpy') 
    t2_func = sp.lambdify((x, y), t2_expr, 'numpy')
    ik_solutions.append((t1_func, t2_func))

# -------------------------------------------------------------
# --- 2. 輸出 Python 函式程式碼 (保持不變，此處輸出到 Console) ---
# -------------------------------------------------------------

print("========================================================================")
print(f"Python 逆運動學函式定義 (L={L_VAL} m, B={B_VAL} m) - 需要 cmath 模組")
print("========================================================================")

for i, sol in enumerate(solutions):
    t1_expr = sol[t1]
    t2_expr = sol[t2]
    
    # 轉換 ccode 輸出為 Python cmath 函式格式
    t1_code = sp.printing.ccode(t1_expr).replace("pow(", "cmath.pow(").replace("sqrt(", "cmath.sqrt(").replace("atan(", "cmath.atan(")
    t2_code = sp.printing.ccode(t2_expr).replace("pow(", "cmath.pow(").replace("sqrt(", "cmath.sqrt(").replace("atan(", "cmath.atan(")
    
    t1_def = f"def t1_sol{i+1}(x, y):\n    return {t1_code}"
    t2_def = f"def t2_sol{i+1}(x, y):\n    return {t2_code}"
    
    print(f"\n# Solution {i+1}:")
    print(t1_def)
    print(t2_def)

print("\n========================================================================")
print("動態模擬開始...")
print("========================================================================")

# -------------------------------------------------------------
# --- 3. 路徑定義：400x400 mm (與基點座標相關) ---
# -------------------------------------------------------------
def generate_path():
    path = []
    SIDE = 0.4      # 矩形邊長 0.4 m
    
    X_START = -0.2
    Y_START = -0.215 
    STEPS = 100
    
    Y_END = Y_START + SIDE
    X_END = X_START + SIDE

    # 1. (-0.2, -0.215) -> (0.2, -0.215)
    for X in np.linspace(X_START, X_END, STEPS):
        path.append((X, Y_START))
    # 2. (0.2, -0.215) -> (0.2, 0.185)
    for Y in np.linspace(Y_START, Y_END, STEPS):
        path.append((X_END, Y))
    # 3. (0.2, 0.185) -> (-0.2, 0.185)
    for X in np.linspace(X_END, X_START, STEPS):
        path.append((X, Y_END))
    # 4. (-0.2, 0.185) -> (-0.2, -0.215)
    for Y in np.linspace(Y_END, Y_START, STEPS):
        path.append((X_START, Y))

    return path

path = generate_path()

# -------------------------------------------------------------
# --- 4. 畫圖初始化與動畫設定 (Matplotlib 動態模擬) ---
# -------------------------------------------------------------

fig, axes = plt.subplots(2, 2, figsize=(14, 14))
axes = axes.flatten()

lines_left = []
lines_right = []
pen_points = []
pen_traces = []
trace_coords = [([], []) for _ in range(len(ik_solutions))]  

equations_titles = [
    r"Elbow Down/Down",
    r"Elbow Down/Up",
    r"Elbow Up/Down",
    r"Elbow Up/Up"
]

for i in range(len(ik_solutions)):
    ax_plt = axes[i]
    # 標題只顯示 Solution 序號和類型，不印在圖面內
    ax_plt.set_title(f"Solution {i+1}: {equations_titles[i]}", fontsize=12)
    
    # 調整座標軸範圍
    ax_plt.set_xlim(-0.5, 0.5) 
    ax_plt.set_ylim(-0.5, 0.5)
    ax_plt.set_aspect('equal')
    ax_plt.grid(True)
    ax_plt.set_xlabel("X coordinate (m)") 
    ax_plt.set_ylabel("Y coordinate (m)") 
    
    # 繪製基點連線 (馬達中心點 A 和 E)
    # A=(-0.2, -0.25), E=(0.2, -0.25)
    ax_plt.plot([ax, ex], [ay, ey], 'k-', lw=3, label='Base') 
    ax_plt.plot([ax, ex], [ay, ey], 'ko', markersize=6) 
    
    # --- 已移除的程式碼 ---
    # ax_plt.text(0.05, 0.95, fr'$L={L_VAL} \text{{ m}}, B={B_VAL} \text{{ m}}$', transform=ax_plt.transAxes, fontsize=10, verticalalignment='top')
    # ax_plt.text(0.05, 0.92, r'Path: $400 \times 400 \text{ mm}$', transform=ax_plt.transAxes, fontsize=9)
    # --- 

    line_left, = ax_plt.plot([], [], 'ro-', lw=2, label='Link 1') 
    line_right, = ax_plt.plot([], [], 'bo-', lw=2, label='Link 2') 
    pen_point, = ax_plt.plot([], [], 'g.', markersize=6, label='Pen Tip (P)')
    pen_trace, = ax_plt.plot([], [], 'g--', lw=1, alpha=0.7, label='Path Trace')

    lines_left.append(line_left)
    lines_right.append(line_right)
    pen_points.append(pen_point)
    pen_traces.append(pen_trace)

def init():
    for i in range(len(ik_solutions)):
        lines_left[i].set_data([], [])
        lines_right[i].set_data([], [])
        pen_points[i].set_data([], [])
        pen_traces[i].set_data([], [])
        trace_coords[i] = ([], []) # 重設軌跡
    return lines_left + lines_right + pen_points + pen_traces

def animate(frame):
    if frame >= len(path):
        return lines_left + lines_right + pen_points + pen_traces

    xi, yi = path[frame]

    for i, (t1_func, t2_func) in enumerate(ik_solutions):
        try:
            theta1_complex = t1_func(xi, yi)
            theta2_complex = t2_func(xi, yi)

            # 確保解是實數
            if abs(theta1_complex.imag) < 1e-6 and abs(theta2_complex.imag) < 1e-6:
                theta1 = theta1_complex.real
                theta2 = theta2_complex.real
            else:
                # 點不可達 (複數解)
                raise ValueError("Unreachable (Complex Solution)")

            # 左連桿 (L1): A -> B -> P
            bx_calc = ax + L_VAL * np.cos(theta1)
            by_calc = ay + L_VAL * np.sin(theta1)
            lines_left[i].set_data([ax, bx_calc, xi], [ay, by_calc, yi])

            # 右連桿 (L2): E -> D -> P
            dx_calc = ex + L_VAL * np.cos(theta2)
            dy_calc = ey + L_VAL * np.sin(theta2)
            lines_right[i].set_data([ex, dx_calc, xi], [ey, dy_calc, yi])
            
            # 筆尖
            pen_points[i].set_data([xi], [yi])

            # 軌跡
            trace_x, trace_y = trace_coords[i]
            trace_x.append(xi)
            trace_y.append(yi)
            pen_traces[i].set_data(trace_x, trace_y)

        except Exception as e:
            # 處理點不可達的情況
            lines_left[i].set_data([ax], [ay]) # 只顯示基點
            lines_right[i].set_data([ex], [ey]) # 只顯示基點
            pen_points[i].set_data([], [])
            # 在軌跡中插入 NaN 來中斷線段
            if len(trace_coords[i][0]) > 0 and not np.isnan(trace_coords[i][0][-1]):
                trace_coords[i][0].append(np.nan)
                trace_coords[i][1].append(np.nan)

    return lines_left + lines_right + pen_points + pen_traces

ani = animation.FuncAnimation(
    fig, animate, init_func=init,
    frames=len(path) + 50,
    interval=30, blit=True
)

plt.suptitle(f"5-bar Linkage Plotter Simulation ($L={L_VAL}, B={B_VAL} \text{{ m}}$) - $400 \times 400 \text{{ mm}}$ Path", fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.95])

# 儲存為 .gif 檔案 (可選)
ani.save('animation_webots.gif', writer='pillow', dpi=80)

plt.show()