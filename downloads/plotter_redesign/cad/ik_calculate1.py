import math

# ----------------- 參數設定 -----------------
L1 = 0.20  # AB
L2 = 0.27  # BC
L3 = 0.27  # CD
L4 = 0.20  # DE
L5 = 0.20  # AE (基座長度)

def inverse_kinematics_5bar(cx: float, cy: float) -> list:
    """
    計算五連桿平面機械臂的逆運動學解 (t1, t2)。
    有四組解，以 [t1, t2] 格式的列表返回所有有效解。
    
    :param cx: 終端點 C 的 x 座標
    :param cy: 終端點 C 的 y 座標
    :return: 包含所有四種配置解的列表 [ [t1_1, t2_1], [t1_2, t2_2], ... ]
    """
    
    # 用於儲存四組解
    solutions = []
    
    # ------------------ 左側連桿 (A-B-C) ------------------
    # A = (0, 0)
    
    # 1. 計算 AC 的長度 L_AC
    L_AC_sq = cx**2 + cy**2
    L_AC = math.sqrt(L_AC_sq)
    
    # 檢查工作空間可達性 (A-B-C 鏈)
    # L1 + L2 > L_AC 且 |L1 - L2| < L_AC
    if L_AC > (L1 + L2) or L_AC < abs(L1 - L2):
        # print("C點超出左側連桿的可達範圍")
        return []

    # 2. 計算 AC 相對於水平線的角度 (α)
    alpha = math.atan2(cy, cx)
    
    # 3. 利用餘弦定理計算 <BAC (β1)
    # cos(β1) = (L1^2 + L_AC^2 - L2^2) / (2 * L1 * L_AC)
    try:
        cos_beta1 = (L1**2 + L_AC_sq - L2**2) / (2 * L1 * L_AC)
        # 確保 cos_beta1 在 [-1, 1] 之間，避免浮點誤差導致 math.acos 錯誤
        cos_beta1 = max(-1.0, min(1.0, cos_beta1)) 
        beta1 = math.acos(cos_beta1)
    except ZeroDivisionError:
        # L_AC = 0, C 點在 A 點上，無意義
        return []

    # 4. 根據 σ1 = +1 和 σ1 = -1 得到 t1 的兩個解
    t1_plus = alpha + beta1    # σ1 = +1
    t1_minus = alpha - beta1   # σ1 = -1
    
    t1_solutions = [t1_plus, t1_minus]
    t1_sigmas = [1, -1] # 紀錄對應的連桿姿態
    
    # ------------------ 右側連桿 (E-D-C) ------------------
    # E = (L5, 0)
    
    # 1. 計算 EC 的長度 L_EC
    Ex = L5
    Ey = 0
    dx = cx - Ex
    dy = cy - Ey
    L_EC_sq = dx**2 + dy**2
    L_EC = math.sqrt(L_EC_sq)

    # 檢查工作空間可達性 (E-D-C 鏈)
    # L4 + L3 > L_EC 且 |L4 - L3| < L_EC
    if L_EC > (L4 + L3) or L_EC < abs(L4 - L3):
        # print("C點超出右側連桿的可達範圍")
        return []

    # 2. 計算 EC 相對於水平線的角度 (γ)
    gamma = math.atan2(dy, dx)
    
    # 3. 利用餘弦定理計算 <DEC (β2)
    # cos(β2) = (L4^2 + L_EC^2 - L3^2) / (2 * L4 * L_EC)
    try:
        cos_beta2 = (L4**2 + L_EC_sq - L3**2) / (2 * L4 * L_EC)
        # 確保 cos_beta2 在 [-1, 1] 之間
        cos_beta2 = max(-1.0, min(1.0, cos_beta2)) 
        beta2 = math.acos(cos_beta2)
    except ZeroDivisionError:
        # L_EC = 0, C 點在 E 點上，無意義
        return []

    # 4. 根據 σ2 = +1 和 σ2 = -1 得到 t2 的兩個解
    # t2 = gamma + sigma2 * beta2 (注意：這邊的 + / - 是取決於連桿定義，
    # 幾何上 <DEC 是在 L_EC 向量的順時針或逆時針方向，
    # 這裡我們使用常用的慣例： t2_plus/minus 對應 D 點兩種可能位置)
    
    t2_plus = gamma + beta2    # σ2 = +1
    t2_minus = gamma - beta2   # σ2 = -1
    
    t2_solutions = [t2_plus, t2_minus]
    t2_sigmas = [1, -1]
    
    # ------------------ 組合四組解 ------------------
    # 組合方式：(t1_plus, t2_plus), (t1_plus, t2_minus), 
    #          (t1_minus, t2_plus), (t1_minus, t2_minus)
    
    # 由於我們需要以四個函數格式返回，這裡將四種組合分開定義
    
    # 解決方案 I: (σ1=+1, σ2=+1) -> t1_plus, t2_plus
    solutions.append({'t1': t1_plus, 't2': t2_plus, 'config': '(+1, +1)'})
    
    # 解決方案 II: (σ1=+1, σ2=-1) -> t1_plus, t2_minus
    solutions.append({'t1': t1_plus, 't2': t2_minus, 'config': '(+1, -1)'})

    # 解決方案 III: (σ1=-1, σ2=+1) -> t1_minus, t2_plus
    solutions.append({'t1': t1_minus, 't2': t2_plus, 'config': '(-1, +1)'})
    
    # 解決方案 IV: (σ1=-1, σ2=-1) -> t1_minus, t2_minus
    solutions.append({'t1': t1_minus, 't2': t2_minus, 'config': '(-1, -1)'})
    
    # 將角度從 (-pi, pi] 範圍轉換到 [0, 2*pi] 範圍 (可選)
    for sol in solutions:
        # 將角度轉換為 [0, 2*pi] 範圍
        sol['t1'] = sol['t1'] % (2 * math.pi)
        sol['t2'] = sol['t2'] % (2 * math.pi)

    return solutions

# --- 範例測試 ---
# 假設 C 點座標在 (0.2, 0.4)
cx_test = 0.2
cy_test = 0.4
all_solutions = inverse_kinematics_5bar(cx_test, cy_test)
print(f"C({cx_test}, {cy_test}) 的所有逆運動學解:")
for sol in all_solutions:
     print(f"Config {sol['config']}: t1={math.degrees(sol['t1']):.2f} deg, t2={math.degrees(sol['t2']):.2f} deg")