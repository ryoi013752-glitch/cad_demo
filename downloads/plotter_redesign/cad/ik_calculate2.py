import math
import numpy as np

# ----------------- 固定參數 (米) -----------------
L1, L2, L3, L4, L5 = 0.20, 0.27, 0.27, 0.20, 0.20
Ex = L5  # E 點的 x 座標 (0.2)

def ik(cx: float, cy: float) -> tuple:
    """
    計算五連桿機械臂在 (+1, -1) 配置下的馬達角度 (t1, t2)。
    
    t1: 左臂 (AB) 角度，配置為向上 (sigma1 = +1)
    t2: 右臂 (ED) 角度，配置為向下 (sigma2 = -1)
    
    :param cx: 終端點 C 的 x 座標
    :param cy: 終端點 C 的 y 座標
    :return: (t1_rad, t2_rad) 角度，若無解則返回 (None, None)
    """
    
    # ------------------ 左側連桿 (A-B-C) 求解 t1 (sigma1 = +1) ------------------
    
    # 1. AC 向量長度的平方和長度
    L_AC_sq = cx**2 + cy**2
    L_AC = math.sqrt(L_AC_sq)
    
    # 2. 檢查工作空間可達性
    if L_AC > (L1 + L2) or L_AC < abs(L1 - L2) or L_AC == 0:
        return (None, None)

    # 3. 計算 AC 角度 (alpha)
    alpha = math.atan2(cy, cx)
    
    # 4. 利用餘弦定理計算 <BAC 角度 (beta1)
    cos_beta1 = (L1**2 + L_AC_sq - L2**2) / (2 * L1 * L_AC)
    
    # 處理浮點誤差，確保 cos 值在 [-1, 1] 範圍內
    cos_beta1 = np.clip(cos_beta1, -1.0, 1.0)
    beta1 = math.acos(cos_beta1)
    
    # 5. t1 = alpha + sigma1 * beta1
    sigma1 = 1
    t1 = alpha + sigma1 * beta1
    
    # ------------------ 右側連桿 (E-D-C) 求解 t2 (sigma2 = -1) ------------------
    
    # 1. EC 向量分量和長度的平方和長度
    dx, dy = cx - Ex, cy
    L_EC_sq = dx**2 + dy**2
    L_EC = math.sqrt(L_EC_sq)

    # 2. 檢查工作空間可達性
    if L_EC > (L4 + L3) or L_EC < abs(L4 - L3) or L_EC == 0:
        return (None, None)

    # 3. 計算 EC 角度 (gamma)
    gamma = math.atan2(dy, dx)
    
    # 4. 利用餘弦定理計算 <DEC 角度 (beta2)
    cos_beta2 = (L4**2 + L_EC_sq - L3**2) / (2 * L4 * L_EC)
    
    # 處理浮點誤差
    cos_beta2 = np.clip(cos_beta2, -1.0, 1.0)
    beta2 = math.acos(cos_beta2)
    
    # 5. t2 = gamma + sigma2 * beta2
    sigma2 = -1
    t2 = gamma + sigma2 * beta2
    
    # 將角度轉換為 [0, 2*pi] 範圍
    t1_rad = t1 % (2 * math.pi)
    t2_rad = t2 % (2 * math.pi)
    
    #return t1_rad, t2_rad # 傳回 radian
    return math.degrees(t1_rad), math.degrees(t2_rad)
    
# 測試點
cx_test = 0.2
cy_test = 0.2

t1_deg, t2_deg = ik(cx_test, cy_test)

if t1_deg is not None:
    print(f"輸入 C({cx_test}, {cy_test}) 的解為:")
    print(f"t1 (左臂) = {t1_deg:.2f} deg")   # 預期值: 84.35 deg
    print(f"t2 (右臂) = {t2_deg:.2f} deg")   # 預期值: 52.60 deg
else:
    print(f"C({cx_test}, {cy_test}) 點在此配置下無法到達。")