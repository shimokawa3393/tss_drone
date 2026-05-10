# quadcopter_sim.py
# クアッドコプター 簡易シミュレーター（Phase 2-2）

import numpy as np
import matplotlib.pyplot as plt

# ---- 時間設定 ----
dt = 0.01       # 時間刻み [s]
t_max = 10.0    # シミュレーション時間 [s]

# ---- PIDゲイン（3軸分） ----
Kp_roll,  Ki_roll,  Kd_roll  = 2.0, 0, 0
Kp_pitch, Ki_pitch, Kd_pitch = 2.0, 0, 0
Kp_yaw,   Ki_yaw,   Kd_yaw   = 2.0, 0, 0

# ---- 目標姿勢 ----
target_roll  = 0.0
target_pitch = 0.0
target_yaw   = 0.0
throttle     = 500.0  # 基準スロットル（RPM的なイメージ）

# ---- 初期姿勢（外乱として傾けておく） ----
roll  = 10.0
pitch = 5.0
yaw   = 3.0

# ---- 積分・前回誤差の初期化 ----
roll_integral,  prev_roll_error  = 0.0, 0.0
pitch_integral, prev_pitch_error = 0.0, 0.0
yaw_integral,   prev_yaw_error   = 0.0, 0.0

# ---- ログ用リスト ----
t_list     = []
roll_list  = []
pitch_list = []
yaw_list   = []
M1_list, M2_list, M3_list, M4_list = [], [], [], []

# ---- メインループ ----
for t in np.arange(0, t_max, dt):
    
    # ---- 外乱設定 ----
    # t=3秒に突風が来てrollに突然+15度の外乱を加える
    if abs(t - 3.0) < dt:
        roll += 15.0

    # センサーノイズ（現実に近づける）
    noise_roll  = np.random.normal(0, 0.1)
    noise_pitch = np.random.normal(0, 0.1)
    noise_yaw   = np.random.normal(0, 0.1)

    measured_roll  = roll  + noise_roll
    measured_pitch = pitch + noise_pitch
    measured_yaw   = yaw   + noise_yaw

    # ---- PID（roll） ----
    roll_error       = target_roll - measured_roll
    roll_integral    += roll_error * dt
    roll_derivative  = (roll_error - prev_roll_error) / dt
    roll_output      = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative
    prev_roll_error  = roll_error

    # ---- PID（pitch） ----
    pitch_error      = target_pitch - measured_pitch
    pitch_integral   += pitch_error * dt
    pitch_derivative = (pitch_error - prev_pitch_error) / dt
    pitch_output     = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative
    prev_pitch_error = pitch_error

    # ---- PID（yaw） ----
    yaw_error        = target_yaw - measured_yaw
    yaw_integral     += yaw_error * dt
    yaw_derivative   = (yaw_error - prev_yaw_error) / dt
    yaw_output       = Kp_yaw * yaw_error + Ki_yaw * yaw_integral + Kd_yaw * yaw_derivative
    prev_yaw_error   = yaw_error

    # ---- モーター出力分配 ----
    # 物理から導いた符号をそのまま使う
    M1 = throttle + roll_output - pitch_output - yaw_output  # 左前
    M2 = throttle - roll_output - pitch_output + yaw_output  # 右前
    M3 = throttle + roll_output + pitch_output + yaw_output  # 左後
    M4 = throttle - roll_output + pitch_output - yaw_output  # 右後

    # モーター出力を0以上にクリップ（マイナス回転はない）
    M1, M2, M3, M4 = max(0, M1), max(0, M2), max(0, M3), max(0, M4)

    # ---- 姿勢の更新（簡易モデル） ----
    # 出力を姿勢変化に変換（比例係数0.01は仮置き）
    roll  += roll_output  * dt * 0.5
    pitch += pitch_output * dt * 0.5
    yaw   += yaw_output   * dt * 0.5

    # ---- ログ ----
    t_list.append(t)
    roll_list.append(roll)
    pitch_list.append(pitch)
    yaw_list.append(yaw)
    M1_list.append(M1)
    M2_list.append(M2)
    M3_list.append(M3)
    M4_list.append(M4)

# ---- グラフ描画 ----
fig, axes = plt.subplots(2, 1, figsize=(10, 8))

# 姿勢
axes[0].plot(t_list, roll_list,  label="Roll")
axes[0].plot(t_list, pitch_list, label="Pitch")
axes[0].plot(t_list, yaw_list,   label="Yaw")
axes[0].axhline(0, color='k', linestyle='--', label="Target")
axes[0].set_ylabel("Attitude [deg]")
axes[0].legend()
axes[0].grid(True)

# モーター出力
axes[1].plot(t_list, M1_list, label="M1 Left Front")
axes[1].plot(t_list, M2_list, label="M2 Right Front")
axes[1].plot(t_list, M3_list, label="M3 Left Rear")
axes[1].plot(t_list, M4_list, label="M4 Right Rear")
axes[1].set_ylabel("Motor Output [RPM-like]")
axes[1].set_xlabel("Time [s]")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig("quadcopter_sim.png")
plt.show()