import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"

df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))
df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))
df_compass = pd.read_csv(os.path.join(d1, "Compass.csv"))
df_gyro = pd.read_csv(os.path.join(d1, "Gyroscope.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0

# Step detection
fs = len(df_acc) / df_acc['t'].iloc[-1]
# compute vertical acc in world
qw = np.interp(df_acc['t'], df_orient['t'], df_orient['qw'])
qx = np.interp(df_acc['t'], df_orient['t'], df_orient['qx'])
qy = np.interp(df_acc['t'], df_orient['t'], df_orient['qy'])
qz = np.interp(df_acc['t'], df_orient['t'], df_orient['qz'])
# normalize
qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2) + 1e-9
qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm

# Acc world z
# R[2,0] = 2(qx*qz - qy*qw), R[2,1] = 2(qy*qz + qx*qw), R[2,2] = 1 - 2(qx^2 + qy^2)
az_w = (2*(qx*qz - qy*qw)*df_acc['x'] + 
        2*(qy*qz + qx*qw)*df_acc['y'] + 
        (1 - 2*(qx**2 + qy**2))*df_acc['z'] - 9.80665)

b, a = signal.butter(2, [1.0/(0.5*fs), 3.2/(0.5*fs)], btype='band')
az_filt = signal.filtfilt(b, a, az_w)
peaks, _ = signal.find_peaks(az_filt, height=0.45, distance=int(0.28 * fs))
step_times = df_acc['t'].iloc[peaks].values
n_steps = len(peaks)

# Compare Compass magneticBearing vs Orientation yaw
orient_yaw = np.interp(step_times, df_orient['t'], df_orient['yaw']) # in radians or deg?
# Check orientation yaw range
print("Orient yaw min, max:", df_orient['yaw'].min(), df_orient['yaw'].max())
print("Compass bearing min, max:", df_compass['magneticBearing'].min(), df_compass['magneticBearing'].max())

compass_bearing = np.interp(step_times, df_compass['t'], df_compass['magneticBearing'])
# Unwrapped yaw in degrees
orient_yaw_deg = np.degrees(np.interp(step_times, df_orient['t'], df_orient['yaw']))

# Inspect heading progression step by step
print(f"Total steps: {n_steps}")

# Detect flights by clustering heading
# Let's print each step's time, compass bearing, orient yaw, gyro_z
gyro_z = np.interp(step_times, df_gyro['t'], df_gyro['z'])
gyro_x = np.interp(step_times, df_gyro['t'], df_gyro['x'])

df_steps = pd.DataFrame({
    'step': np.arange(1, n_steps + 1),
    'time': np.round(step_times, 2),
    'compass': np.round(compass_bearing, 1),
    'orient_yaw_deg': np.round(orient_yaw_deg, 1),
    'gyro_z': np.round(gyro_z, 3),
    'gyro_x': np.round(gyro_x, 3),
})

# Save to csv for inspection
df_steps.to_csv(r"c:\New PJ\OFF-GRID SURVIVAL\scripts\dataset1_steps_detailed.csv", index=False)
print("Saved steps detailed to dataset1_steps_detailed.csv")
