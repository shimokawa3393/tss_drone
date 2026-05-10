# quadcopter_sim_compare.py
# 3パターンの姿勢とモーター出力を縦6段で比較

import numpy as np
import matplotlib.pyplot as plt

# ---- 時間設定 ----
dt = 0.01
t_max = 10.0

# ---- 比較する3パターンのゲイン設定 ----
patterns = [
    {"label": "Kp only", "Kp": 2.0, "Ki": 0.0, "Kd": 0.0},
    {"label": "Kp + Ki", "Kp": 2.0, "Ki": 0.1, "Kd": 0.0},
    {"label": "Kp + Kd", "Kp": 2.0, "Ki": 0.0, "Kd": 0.5},
]

fig, axes = plt.subplots(6, 1, figsize=(10, 24))

for i, p in enumerate(patterns):

    # ---- 初期姿勢・積分・前回誤差 ----
    roll, pitch, yaw                         = 10.0, 5.0, 3.0
    roll_integral, pitch_integral, yaw_integral     = 0.0, 0.0, 0.0
    prev_roll_error, prev_pitch_error, prev_yaw_error = 0.0, 0.0, 0.0
    throttle                                 = 500.0

    t_list                           = []
    roll_list, pitch_list, yaw_list  = [], [], []
    M1_list, M2_list, M3_list, M4_list = [], [], [], []

    for t in np.arange(0, t_max, dt):

        # ---- 外乱（t=3秒に3軸全部） ----
        if abs(t - 3.0) < dt:
            roll  += 15.0
            pitch += 10.0
            yaw   += 5.0

        # ---- センサーノイズ ----
        measured_roll  = roll  + np.random.normal(0, 0.1)
        measured_pitch = pitch + np.random.normal(0, 0.1)
        measured_yaw   = yaw   + np.random.normal(0, 0.1)

        # ---- PID（roll） ----
        roll_error      = 0.0 - measured_roll
        roll_integral   += roll_error * dt
        roll_derivative = (roll_error - prev_roll_error) / dt
        roll_output     = p["Kp"] * roll_error + p["Ki"] * roll_integral + p["Kd"] * roll_derivative
        prev_roll_error = roll_error

        # ---- PID（pitch） ----
        pitch_error      = 0.0 - measured_pitch
        pitch_integral   += pitch_error * dt
        pitch_derivative = (pitch_error - prev_pitch_error) / dt
        pitch_output     = p["Kp"] * pitch_error + p["Ki"] * pitch_integral + p["Kd"] * pitch_derivative
        prev_pitch_error = pitch_error

        # ---- PID（yaw） ----
        yaw_error      = 0.0 - measured_yaw
        yaw_integral   += yaw_error * dt
        yaw_derivative = (yaw_error - prev_yaw_error) / dt
        yaw_output     = p["Kp"] * yaw_error + p["Ki"] * yaw_integral + p["Kd"] * yaw_derivative
        prev_yaw_error = yaw_error

        # ---- モーター分配 ----
        M1 = throttle + roll_output - pitch_output - yaw_output  # 左前
        M2 = throttle - roll_output - pitch_output + yaw_output  # 右前
        M3 = throttle + roll_output + pitch_output + yaw_output  # 左後
        M4 = throttle - roll_output + pitch_output - yaw_output  # 右後
        M1, M2, M3, M4 = max(0, M1), max(0, M2), max(0, M3), max(0, M4)

        # ---- 姿勢更新 ----
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

    # ---- 姿勢グラフ ----
    ax_att = axes[i * 2]
    ax_att.plot(t_list, roll_list,  label="Roll")
    ax_att.plot(t_list, pitch_list, label="Pitch")
    ax_att.plot(t_list, yaw_list,   label="Yaw")
    ax_att.axhline(0, color='k', linestyle='--', label="Target")
    ax_att.axvline(3.0, color='gray', linestyle=':', label="Disturbance")
    ax_att.set_title(f"{p['label']} - Attitude")
    ax_att.set_ylabel("Attitude [deg]")
    ax_att.legend()
    ax_att.grid(True)

    # ---- モーターグラフ ----
    ax_mot = axes[i * 2 + 1]
    ax_mot.plot(t_list, M1_list, label="M1 Left Front")
    ax_mot.plot(t_list, M2_list, label="M2 Right Front")
    ax_mot.plot(t_list, M3_list, label="M3 Left Rear")
    ax_mot.plot(t_list, M4_list, label="M4 Right Rear")
    ax_mot.axvline(3.0, color='gray', linestyle=':', label="Disturbance")
    ax_mot.set_title(f"{p['label']} - Motor Output")
    ax_mot.set_ylabel("Motor Output [a.u.]")
    ax_mot.legend()
    ax_mot.grid(True)

axes[5].set_xlabel("Time [s]")

plt.tight_layout()
plt.savefig("pid_comparison.png")
plt.show()