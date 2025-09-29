import cmath
import math

# Solution 3 的函式定義 (已修正 cmath.pow -> **)
def t1_sol3(x, y):
    numerator = (1280.0 * y + cmath.sqrt(-4000000.0 * (x ** 4) - 3200000.0 * (x ** 3) - 8000000.0 * (x ** 2) * (y ** 2) - 4000000.0 * (x ** 2) * y + 178400.0 * (x ** 2) - 3200000.0 * x * (y ** 2) - 1600000.0 * x * y + 327360.0 * x - 4000000.0 * (y ** 4) - 4000000.0 * (y ** 3) - 181600.0 * (y ** 2) + 409200.0 * y + 125911.0) + 320.0)
    denominator = (2000.0 * (x ** 2) + 2080.0 * x + 2000.0 * (y ** 2) + 1000.0 * y + 461.0)
    
    # 確保分母不為零，雖然在逆運動學中不太可能
    if abs(denominator) < 1e-9:
        raise ValueError("Denominator is near zero.")
        
    return 2.0 * cmath.atan(numerator / denominator)

def t2_sol3(x, y):
    numerator = (1280.0 * y - cmath.sqrt(-4000000.0 * (x ** 4) + 3200000.0 * (x ** 3) - 8000000.0 * (x ** 2) * (y ** 2) - 4000000.0 * (x ** 2) * y + 178400.0 * (x ** 2) + 3200000.0 * x * (y ** 2) + 1600000.0 * x * y - 327360.0 * x - 4000000.0 * (y ** 4) - 4000000.0 * (y ** 3) - 181600.0 * (y ** 2) + 409200.0 * y + 125911.0) + 320.0)
    denominator = (2000.0 * (x ** 2) + 480.0 * x + 2000.0 * (y ** 2) + 1000.0 * y - 51.0)
    
    # 確保分母不為零
    if abs(denominator) < 1e-9:
        raise ValueError("Denominator is near zero.")
        
    return 2.0 * cmath.atan(numerator / denominator)

# 輸入座標 (m)
x_val = 0.2
y_val = 0.185

try:
    # 計算角度 (弧度)
    t1_rad = t1_sol3(x_val, y_val)
    t2_rad = t2_sol3(x_val, y_val)

    # 檢查是否為實數解並轉換為角度 (degrees)
    # 由於 SymPy 輸出的方程式是基於 cmath，結果 t1_rad/t2_rad 可能是 complex 類型。
    # 檢查虛部是否接近零。
    if abs(t1_rad.imag) < 1e-9:
        t1_deg = math.degrees(t1_rad.real)
    else:
        t1_deg = f"Complex ({t1_rad})"

    if abs(t2_rad.imag) < 1e-9:
        t2_deg = math.degrees(t2_rad.real)
    else:
        t2_deg = f"Complex ({t2_rad})"

    print(f"--- Solution 3 at P({x_val}, {y_val}) ---")
    print(f"t1 (Radians): {t1_rad}")
    print(f"t2 (Radians): {t2_rad}")
    print("------------------------------------------")
    print(f"t1 (Degrees): {t1_deg:.4f} degrees" if isinstance(t1_deg, float) else f"t1 (Degrees): {t1_deg}")
    print(f"t2 (Degrees): {t2_deg:.4f} degrees" if isinstance(t2_deg, float) else f"t2 (Degrees): {t2_deg}")

except Exception as e:
    print(f"Calculation failed: {e}")