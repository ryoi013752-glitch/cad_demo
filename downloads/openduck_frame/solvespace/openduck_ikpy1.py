# pip install ikpy numpy
# 尚未設定第一軸不可移動
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

# 建立機器人腿的 kinematic chain
leg_chain = Chain(name='duck_leg', links=[
    OriginLink(),  # Base link

    # Hip Yaw - rot_z
    URDFLink(
        name="hip_yaw",
        origin_translation=[0, 0, 0.03472],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),

    # Hip Roll - rot_y
    URDFLink(
        name="hip_roll",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),

    # Hip Pitch - rot_x
    URDFLink(
        name="hip_pitch",
        origin_translation=[0, 0, -0.07827],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    ),

    # Knee Pitch - rot_x
    URDFLink(
        name="knee_pitch",
        origin_translation=[0, 0, -0.07827],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    ),

    # Ankle Pitch - rot_x
    URDFLink(
        name="ankle_pitch",
        origin_translation=[0, 0, -0.03132],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    ),
])

# 設定足端目標位置（世界座標）
target_position = [0.05, 0.02, -0.15]  # XYZ in meters

# 移除建立 target_frame 的步驟，直接使用 target_position

# 初始角度（全 0）。
initial_positions = [0] * len(leg_chain.links)

# 執行 IK 解算
# ★ 直接傳入 target_position (3D 向量)，讓 ikpy 在內部建立 4x4 矩陣
ik_solution = leg_chain.inverse_kinematics(
    target_position,  # 直接傳遞 3D 位置
    initial_position=initial_positions
)

# 輸出每個關節角度（弧度）
for i, link in enumerate(leg_chain.links[1:]):  # 忽略 base link (i=0)
    print(f"{link.name}: {ik_solution[i+1]:.4f} rad")