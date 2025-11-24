# w13.py —— 每代從原始蹲姿開始 + 印參數 + 第一步穩定度評分（100% 可跑）
from controller import Supervisor
import math
import random
import signal
import sys

TIME_STEP = 16
supervisor = Supervisor()

# 取得所有馬達與感測器
motors = {name: supervisor.getDevice(name) for name in 
          ["t1","t2","t3","t4","t5","rt1","rt2","rt3","rt4","rt5"]}
for m in motors.values():
    m.setVelocity(8.5)

imu = supervisor.getDevice("inertial_unit")
imu.enable(TIME_STEP)
gps = supervisor.getDevice("gps")
gps.enable(TIME_STEP)

# 原始初始蹲姿（你最穩的那組）
INITIAL_POSE = {
    "t1": 0, "rt1": 0, "t2": 0, "rt2": 0,
    "t3": math.radians(50), "rt3": math.radians(50),
    "t4": math.radians(-100), "rt4": math.radians(-100),
    "t5": math.radians(52), "rt5": math.radians(52)
}

# GA 參數範圍
PARAM_RANGES = {
    "PITCH_OFFSET": [-3.0, -0.3],
    "BALANCE_GAIN": [0.3, 3.0],
    "KP_PITCH": [0.5, 4.0],
    "FWD_SWING": [0.03, 0.16],
    "GAIT_PERIOD": [0.6, 1.8]
}
PARAM_KEYS = list(PARAM_RANGES.keys())

# 進化記錄
best_gene = None
best_score = 0.0
generation = 0

# 每代都從 .wbt 初始狀態 + 初始蹲姿開始
def reset_to_initial():
    supervisor.simulationReset()           # 真正重啟世界
    supervisor.simulationResetPhysics()
    supervisor.step(100)                   # 等待載入

    # 強制回到初始蹲姿
    for name, angle in INITIAL_POSE.items():
        motors[name].setPosition(angle)
    motors["t2"].setPosition(0)
    motors["rt2"].setPosition(0)

    # 等待完全穩定（6秒）
    for _ in range(375):
        if supervisor.step(TIME_STEP) == -1:
            return False
    return True

# 單次評估：只看「第一步是否成功跨出且穩定」
def evaluate_first_step(params):
    # 印出當前參數
    print(f"\n=== 第 {generation+1} 代 測試參數 ===")
    for k, v in params.items():
        print(f"  {k}: {v:.5f}")

    if not reset_to_initial():
        return 0.0

    # 讀取初始高度
    z_initial = gps.getValues()[2]
    z_target = z_initial
    phase = 0.0
    prev_z_error = prev_pitch_error = 0.0
    dt = TIME_STEP / 1000.0

    # 只跑 1.5 秒（足夠判斷第一步）
    max_test_time = 1.5
    sim_time = 0.0
    stable_steps = 0
    total_steps = 0

    while sim_time < max_test_time:
        if supervisor.step(TIME_STEP) == -1:
            break

        current_z = gps.getValues()[2]
        if current_z < z_initial - 0.08:  # 倒地
            print(f"  → 倒地！存活 {sim_time:.3f}s")
            return sim_time

        # Z 控制
        z_error = z_target - current_z
        z_error_rate = (z_error - prev_z_error) / dt
        prev_z_error = z_error
        z_correction = 42.0 * z_error + 9.0 * z_error_rate

        # Pitch 控制
        roll, pitch, _ = imu.getRollPitchYaw()
        pitch_error = pitch - params["PITCH_OFFSET"]
        pitch_error_rate = (pitch_error - prev_pitch_error) / dt
        prev_pitch_error = pitch_error
        pitch_correction = -(params["KP_PITCH"] * pitch_error + 0.1 * pitch_error_rate) * params["BALANCE_GAIN"]

        # 從初始姿態開始
        pos = INITIAL_POSE.copy()

        phase += dt / params["GAIT_PERIOD"]
        t = (phase % 1.0) * 2 * math.pi

        lift_l = max(0, math.sin(t)) * 0.025
        lift_r = max(0, math.sin(t + math.pi)) * 0.025
        fwd = math.cos(t) * params["FWD_SWING"]

        pos["t3"]  += math.radians(fwd * 850) + pitch_correction
        pos["t4"]  += math.radians(lift_l * 1000)
        pos["t5"]  -= math.radians(fwd * 850) - pitch_correction
        pos["t1"]  += math.sin(t - 0.3) * 0.35

        pos["rt3"] += math.radians(fwd * 850 * -1) + pitch_correction
        pos["rt4"] += math.radians(lift_r * 1000)
        pos["rt5"] -= math.radians(fwd * 850 * -1) - pitch_correction
        pos["rt1"] += math.sin(t + math.pi - 0.3) * 0.35

        pos["t5"]  += z_correction
        pos["rt5"] += z_correction

        for name, p in pos.items():
            motors[name].setPosition(p)

        # 穩定度判斷（Z 高度波動小）
        if abs(current_z - z_target) < 0.015:
            stable_steps += 1
        total_steps += 1

        sim_time += dt

    stability_ratio = stable_steps / total_steps if total_steps > 0 else 0
    score = sim_time + stability_ratio * 2.0  # 存活時間 + 穩定度加權
    print(f"  → 第一步完成！存活 {sim_time:.3f}s，穩定度 {stability_ratio:.1%}，得分 {score:.3f}")
    return score

# 無限進化
population = [[random.uniform(*PARAM_RANGES[k]) for k in PARAM_KEYS] for _ in range(8)]

def ctrl_c_handler(sig, frame):
    print("\n進化停止！最終最佳參數：")
    if best_gene:
        for k, v in zip(PARAM_KEYS, best_gene):
            print(f"{k} = {v:.6f}")
    sys.exit(0)
signal.signal(signal.SIGINT, ctrl_c_handler)

print("每代都從原始蹲姿開始 + 印參數 + 第一步穩定度評分！（按 Ctrl+C 停止）")

while True:
    generation += 1
    scores = []
    for gene in population:
        params = dict(zip(PARAM_KEYS, gene))
        score = evaluate_first_step(params)
        scores.append(score)

    best_idx = scores.index(max(scores))
    if scores[best_idx] > best_score:
        best_score = scores[best_idx]
        best_gene = population[best_idx]
        print(f"第 {generation} 代 → 新紀錄！得分 {best_score:.3f}")

    # 精英保留 + 變異
    new_pop = [population[best_idx]]
    while len(new_pop) < 8:
        child = [best_gene[i] + random.uniform(-0.3, 0.3)*(PARAM_RANGES[PARAM_KEYS[i]][1]-PARAM_RANGES[PARAM_KEYS[i]][0])
                 for i in range(len(best_gene))]
        child = [max(min(v, PARAM_RANGES[k][1]), PARAM_RANGES[k][0]) for v, k in zip(child, PARAM_KEYS)]
        new_pop.append(child)
    population = new_pop