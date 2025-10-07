import sympy as sp
import math

# --- 定義符號與常數 ---
x, y = sp.symbols('x y', real=True)
t1, t2 = sp.symbols('t1 t2', real=True)
L1 = 20
L2 = 27
B = 20

# --- 定義逆向運動學方程式 ---
eq1 = sp.Eq((L1 * sp.cos(t1) - x)**2 + (L1 * sp.sin(t1) - y)**2, L2**2)
eq2 = sp.Eq((B - L1 * sp.cos(t2) - x)**2 + (L1 * sp.sin(t2) - y)**2, L2**2)

# --- 解出四組 symbolic 解 ---
solutions = sp.solve([eq1, eq2], (t1, t2), dict=True)

# --- 弧度轉角度 ---
def rad_to_deg(rad):
    return math.degrees(rad)

# --- 篩選合法解並印出程式碼 ---
print("合法解（角度 < 90°）:")
valid_funcs = []

for i, sol in enumerate(solutions, 1):
    try:
        t1_expr = sol[t1]
        t2_expr = sol[t2]

        # 用 lambdify 產生可執行函式
        f_t1 = sp.lambdify((x, y), t1_expr, modules='math')
        f_t2 = sp.lambdify((x, y), t2_expr, modules='math')

        t1_val = f_t1(20, 40)
        t2_val = f_t2(20, 40)

        t1_deg = rad_to_deg(t1_val)
        t2_deg = rad_to_deg(t2_val)

        if 0 <= t1_deg <= 90 and 0 <= t2_deg <= 90:
            print(f"sol{i}: t1 = {t1_deg:.2f}°, t2 = {t2_deg:.2f}°")

            # 儲存合法函式（執行用）
            valid_funcs.append((f"sol{i}", f_t1, f_t2))

            # 輸出對應 Python 函式碼
            print(f"\n# 🔹 Python 函式：sol{i}")
            print(f"def sol{i}(x_val, y_val):")
            print(f"    from math import cos, sin, sqrt, atan2, acos")
            print(f"    t1 = {sp.python(t1_expr)}")
            print(f"    t2 = {sp.python(t2_expr)}")
            print(f"    t1 = t1.subs({{'x': x_val, 'y': y_val}}).evalf()")
            print(f"    t2 = t2.subs({{'x': x_val, 'y': y_val}}).evalf()")
            print(f"    return float(t1), float(t2)\n")

    except Exception as e:
        continue

# --- 可使用函式名稱（執行用） ---
print("\n--- 測試合法函式 ---")
for name, f1, f2 in valid_funcs:
    t1, t2 = f1(20, 40), f2(20, 40)
    print(f"{name}(20, 40) → t1 = {rad_to_deg(t1):.2f}°, t2 = {rad_to_deg(t2):.2f}°")
