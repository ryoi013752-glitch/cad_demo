# pip install ikpy numpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

# 總共有 6 個鏈結：
# [0: OriginLink (fixed), 1: hip_yaw, 2: hip_roll, 3: hip_pitch, 4: knee_pitch, 5: ankle_pitch]
# 只有索引 1 到 5 是可動的 (True)
active_mask = [False, True, True, True, True, True]

# 建立機器人腿的 kinematic chain
leg_chain = Chain(
    name='duck_leg', 
    links=[
        OriginLink(),  # 索引 0: Base link - 設為 False

        # Hip Yaw - rot_z (索引 1)
        URDFLink(
            name="hip_yaw",
            origin_translation=[0, 0, 0.03472],  # 修正參數名稱
            origin_orientation=[0, 0, 0],       # 修正參數名稱
            rotation=[0, 0, 1]
        ),

        # Hip Roll - rot_y (索引 2)
        URDFLink(
            name="hip_roll",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]
        ),

        # Hip Pitch - rot_x (索引 3)
        URDFLink(
            name="hip_pitch",
            origin_translation=[0, 0, -0.07827],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0]
        ),

        # Knee Pitch - rot_x (索引 4)
        URDFLink(
            name="knee_pitch",
            origin_translation=[0, 0, -0.07827],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0]
        ),

        # Ankle Pitch - rot_x (索引 5)
        URDFLink(
            name="ankle_pitch",
            origin_translation=[0, 0, -0.03132],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0]
        ),
    ],
    # 傳入 active_links_mask 以消除 OriginLink 的警告
    active_links_mask=active_mask 
)

# 設定足端目標位置（世界座標）
target_position = [0.05, 0.02, -0.15]  # XYZ in meters

# 初始角度（全 0）。長度必須等於鏈結總數 (6)。
initial_positions = [0] * len(leg_chain.links)

# 執行 IK 解算
# 直接傳入 3D 向量，以避免之前遇到的 ValueError
ik_solution = leg_chain.inverse_kinematics(
    target_position,
    initial_position=initial_positions
)

# 輸出每個關節角度（弧度）
for i, link in enumerate(leg_chain.links[1:]):  # 忽略 base link (i=0)
    print(f"{link.name}: {ik_solution[i+1]:.4f} rad")