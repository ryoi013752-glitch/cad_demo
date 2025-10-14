import cv2
import numpy as np
import matplotlib.pyplot as plt

# 參數設定
image_path = "apple-clip-art.jpg"  # 請改成你的圖檔路徑
target_center = np.array([0.1, 0.3])  # 目標圓心
target_radius = 0.1                   # 目標半徑

# 讀取圖像（灰階）
img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# 二值化，黑線變前景（白底黑線變黑底白線）
_, binary = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)

# 找輪廓 (只取外部輪廓)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
main_contour = max(contours, key=cv2.contourArea)

# 取 50 個點，從原始輪廓等距抽樣（不簡化）
N = 50
if len(main_contour) > N:
    indices = np.linspace(0, len(main_contour) - 1, N, dtype=int)
    sampled = main_contour[indices]
else:
    sampled = main_contour

pts = sampled[:, 0, :]  # 提取 (x, y)

# 修正y軸方向（OpenCV y軸向下，要轉成向上）
h, w = binary.shape
pts[:, 1] = h - pts[:, 1]

# 中心化座標（以重心為中心）
center = np.mean(pts, axis=0)
pts_centered = pts - center

# 正規化至最大距離1的單位圓
scale = np.max(np.linalg.norm(pts_centered, axis=1))
pts_normalized = pts_centered / scale

# 縮放並平移到目標圓內
pts_scaled = pts_normalized * target_radius + target_center

# 繪製結果
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.title("Apple")
plt.imshow(binary, cmap='gray')
plt.plot(pts[:, 0], h - pts[:, 1], 'r.-')  # 輪廓點疊加

plt.subplot(1, 2, 2)
plt.title("Points of Apple (50點)")
plt.plot(pts_scaled[:, 0], pts_scaled[:, 1], 'r.-')
plt.axis("equal")
plt.grid(True)
plt.xlabel("X")
plt.ylabel("Y")

plt.show()

# 匯出點座標檔案，可用於Plotter控制
'''
np.savetxt("apple_contour_50pts.txt", pts_scaled, fmt="%.5f", header="x y", comments='')
print("已匯出 50 點座標到 apple_contour_50pts.txt")
'''
