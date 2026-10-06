import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

path = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_acc = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
df_att = pd.read_csv(os.path.join(path, "Orientation.csv"))
df_gyro = pd.read_csv(os.path.join(path, "Gyroscope.csv"))
df_compass = pd.read_csv(os.path.join(path, "Compass.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_att['t'] = df_att['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0

# Interpolate attitude
qw = np.interp(df_acc['t'], df_att['t'], df_att['qw'])
qx = np.interp(df_acc['t'], df_att['t'], df_att['qx'])
qy = np.interp(df_acc['t'], df_att['t'], df_att['qy'])
qz = np.interp(df_acc['t'], df_att['t'], df_att['qz'])
qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm

# Project acceleration onto gravity vector
# Body frame gravity: g_b = R_nb^T * [0, 0, 1]
# a_v = a_b . g_b - 9.80665
gx_b = 2.0 * (qx*qz - qw*qy)
gy_b = 2.0 * (qy*qz + qw*qx)
gz_b = 1.0 - 2.0 * (qx*qx + qy*qy)

ax = df_acc['x'].values
ay = df_acc['y'].values
az = df_acc['z'].values

a_v_total = ax * gx_b + ay * gy_b + az * gz_b
a_v_dyn = a_v_total - 9.80665
df_acc['a_v'] = a_v_dyn

fs = len(df_acc) / df_acc['t'].iloc[-1]
nyq = 0.5 * fs
b, a = signal.butter(2, [1.0 / nyq, 3.2 / nyq], btype='band')
a_v_filt = signal.filtfilt(b, a, a_v_dyn)

peaks, _ = signal.find_peaks(a_v_filt, height=0.5, distance=int(0.3 * fs))
step_times = df_acc['t'].iloc[peaks].values
print(f"Total steps found: {len(peaks)}")

# Compass heading over time
compass_interp = np.interp(df_acc['t'], df_compass['t'], df_compass['magneticBearing'])

# Detect when the user reached the top and turned around!
# Let's inspect the compass heading in 5s windows
print("\nCompass heading every 5 seconds:")
for t in range(0, int(df_acc['t'].iloc[-1]), 5):
    sub = df_compass[(df_compass['t'] >= t) & (df_compass['t'] < t + 5)]
    b_mean = sub['magneticBearing'].mean()
    b_std = sub['magneticBearing'].std()
    p_mean = df_att[(df_att['t'] >= t) & (df_att['t'] < t + 5)]['pitch'].mean()
    # P2P of a_v
    sub_acc = df_acc[(df_acc['t'] >= t) & (df_acc['t'] < t + 5)]['a_v']
    p2p = sub_acc.max() - sub_acc.min() if len(sub_acc) > 0 else 0
    print(f"t={t:2d}s - {t+5:2d}s: Bearing={b_mean:5.1f} deg (std={b_std:4.1f}), Pitch={np.degrees(p_mean):5.1f} deg, a_v P2P={p2p:4.1f} m/s^2")
