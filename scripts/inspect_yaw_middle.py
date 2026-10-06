import os
import pandas as pd
import numpy as np

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_gyro = pd.read_csv(os.path.join(d1, "Gyroscope.csv"))
df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))
df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0

gz = df_gyro['z'].values
t_gyro = df_gyro['t'].values
dt = np.diff(t_gyro)
yaw_rel = np.zeros(len(t_gyro))
yaw_rel[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)

# Step detection
from scipy import signal
fs = len(df_acc) / df_acc['t'].iloc[-1]
qw = np.interp(df_acc['t'], df_orient['t'], df_orient['qw'])
qx = np.interp(df_acc['t'], df_orient['t'], df_orient['qx'])
qy = np.interp(df_acc['t'], df_orient['t'], df_orient['qy'])
qz = np.interp(df_acc['t'], df_orient['t'], df_orient['qz'])
qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2) + 1e-9
qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm

az_w = (2*(qx*qz - qy*qw)*df_acc['x'] + 
        2*(qy*qz + qx*qw)*df_acc['y'] + 
        (1 - 2*(qx**2 + qy**2))*df_acc['z'] - 9.80665)

b, a = signal.butter(2, [1.0/(0.5*fs), 3.2/(0.5*fs)], btype='band')
az_filt = signal.filtfilt(b, a, az_w)
peaks, _ = signal.find_peaks(az_filt, height=0.42, distance=int(0.28 * fs))
step_times = df_acc['t'].iloc[peaks].values

step_yaw = np.interp(step_times, t_gyro, yaw_rel)

for k in range(len(step_times)):
    if k % 5 == 0 or (step_times[k] > 18 and step_times[k] < 45):
        print(f"Step {k+1:3d} | t={step_times[k]:4.1f}s | Yaw={step_yaw[k]:6.1f}° | Yaw%180={step_yaw[k]%180:5.1f}°")
