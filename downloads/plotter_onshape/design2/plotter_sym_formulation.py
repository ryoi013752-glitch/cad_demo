import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cmath

# --- 定義符號與參數 ---
x, y = sp.symbols('x y', real=True)
t1, t2 = sp.symbols('t1 t2', real=True)

L = 320
B = 400

# --- 定義逆運動學方程式 ---
eq1 = sp.Eq((L * sp.cos(t1) - x)**2 + (L * sp.sin(t1) - y)**2, L**2)
eq2 = sp.Eq((B - L * sp.cos(t2) - x)**2 + (L * sp.sin(t2) - y)**2, L**2)
solutions = sp.solve([eq1, eq2], (t1, t2), dict=True)

# --- 印出 Python 函數格式（cmath） ---
print("------ 逆向運動學解的 Python 函數格式（cmath）------\n")

def convert_expr_to_cmath(expr):
    """將 sympy 表達式轉為 cmath 格式字串"""
    expr_str = str(expr)
    expr_str = expr_str.replace('atan2', 'cmath.atan2')
    expr_str = expr_str.replace('atan', 'cmath.atan')
    expr_str = expr_str.replace('sqrt', 'cmath.sqrt')
    expr_str = expr_str.replace('sin', 'cmath.sin')
    expr_str = expr_str.replace('cos', 'cmath.cos')
    return expr_str

for i, sol in enumerate(solutions):
    t1_expr_str = convert_expr_to_cmath(sol[t1])
    t2_expr_str = convert_expr_to_cmath(sol[t2])
    print(f"# --- Solution {i+1} ---")
    print(f"def t1_sol{i+1}(x, y): return {t1_expr_str}")
    print(f"def t2_sol{i+1}(x, y): return {t2_expr_str}")
    print()

# --- 將解析解轉為可執行函數 ---
ik_solutions = []
for sol in solutions:
    t1_func = sp.lambdify((x, y), sol[t1], modules='numpy')
    t2_func = sp.lambdify((x, y), sol[t2], modules='numpy')
    ik_solutions.append((t1_func, t2_func))

# --- 建立路徑 (正方形在 (0,30)-(400,430)) ---
def generate_path():
    t = np.linspace(0, 1, 400)
    x_path = np.piecewise(t,
        [t < 0.25, (t >= 0.25) & (t < 0.5), (t >= 0.5) & (t < 0.75), t >= 0.75],
        [lambda t: 1600 * t,
         lambda t: 400,
         lambda t: 400 - 1600 * (t - 0.5),
         lambda t: 0])
    
    y_path = np.piecewise(t,
        [t < 0.25, (t >= 0.25) & (t < 0.5), (t >= 0.5) & (t < 0.75), t >= 0.75],
        [lambda t: 30,
         lambda t: 1600 * (t - 0.25) + 30,
         lambda t: 430,
         lambda t: 430 - 1600 * (t - 0.75)])
    
    return list(zip(x_path, y_path))

path = generate_path()

# --- 動畫初始化 ---
fig, axes = plt.subplots(2, 2, figsize=(10, 10))
axes = axes.flatten()
lines = []
pen_points = []
pen_traces = [[] for _ in range(4)]

for ax in axes:
    ax.set_xlim(-100, 600)
    ax.set_ylim(-100, 600)
    ax.set_aspect('equal')
    ax.grid(True)
    line, = ax.plot([], [], 'o-', lw=2)
    pen, = ax.plot([], [], 'go', markersize=4)
    trace, = ax.plot([], [], 'g--', lw=1, alpha=0.5)
    lines.append((line, trace))
    pen_points.append(pen)

def init():
    for line, trace in lines:
        line.set_data([], [])
        trace.set_data([], [])
    for pen in pen_points:
        pen.set_data([], [])
    return sum(([l, t] for l, t in lines), []) + pen_points

def animate(frame):
    if frame >= len(path):
        return []
    xi, yi = path[frame]
    for i, (t1_func, t2_func) in enumerate(ik_solutions):
        try:
            theta1 = t1_func(xi, yi)
            theta2 = t2_func(xi, yi)

            A = (0, 0)
            B_ = (B, 0)
            C = (L * np.cos(theta1), L * np.sin(theta1))
            D = (B - L * np.cos(theta2), L * np.sin(theta2))
            P = (xi, yi)

            xs = [A[0], C[0], P[0], D[0], B_[0]]
            ys = [A[1], C[1], P[1], D[1], B_[1]]
            lines[i][0].set_data(xs, ys)
            pen_points[i].set_data([P[0]], [P[1]])

            pen_traces[i].append(P)
            trace_xs, trace_ys = zip(*pen_traces[i])
            lines[i][1].set_data(trace_xs, trace_ys)

        except Exception as e:
            print(f"[Solution {i+1}] Error at frame {frame}: {e}")
    return sum(([l, t] for l, t in lines), []) + pen_points

ani = animation.FuncAnimation(
    fig, animate, init_func=init,
    frames=len(path), interval=30, blit=True
)

plt.suptitle("五連桿繪圖機動畫 - 四解模擬", fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()
