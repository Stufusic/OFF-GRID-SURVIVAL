import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"

df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))
df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))
df_gyro = pd.read_csv(os.path.join(d1, "Gyroscope.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0

fs = len(df_acc) / df_acc['t'].iloc[-1]

# World vertical acceleration
qw = np.interp(df_acc['t'], df_orient['t'], df_orient['qw'])
qx = np.interp(df_acc['t'], df_orient['t'], df_orient['qx'])
qy = np.interp(df_acc['t'], df_orient['t'], df_orient['qy'])
qz = np.interp(df_acc['t'], df_orient['t'], df_orient['qz'])
qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2) + 1e-9
qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm

az_w = (2*(qx*qz - qy*qw)*df_acc['x'] + 
        2*(qy*qz + qx*qw)*df_acc['y'] + 
        (1 - 2*(qx**2 + qy**2))*df_acc['z'] - 9.80665)

# Bandpass filter
b, a = signal.butter(2, [1.0/(0.5*fs), 3.5/(0.5*fs)], btype='band')
az_filt = signal.filtfilt(b, a, az_w)

# Find all peaks with lower threshold to see if descent steps were missed
peaks_low, _ = signal.find_peaks(az_filt, height=0.30, distance=int(0.25 * fs))
peaks_orig, _ = signal.find_peaks(az_filt, height=0.45, distance=int(0.28 * fs))

t_low = df_acc['t'].iloc[peaks_low].values
t_orig = df_acc['t'].iloc[peaks_orig].values

print(f"Total steps with height=0.45: {len(peaks_orig)}")
print(f"Total steps with height=0.30: {len(peaks_low)}")

# Compare steps before t=40s vs after t=40s
print("Before t=40s: orig =", np.sum(t_orig < 40.0), ", low =", np.sum(t_low < 40.0))
print("After  t=40s: orig =", np.sum(t_orig >= 40.0), ", low =", np.sum(t_low >= 40.0))

# Plot az_filt over time with peaks marked
plt.figure(figsize=(14, 5))
plt.plot(df_acc['t'], az_filt, label='az_world filtered', color='black', lw=0.8)
plt.plot(t_orig, az_filt[peaks_orig], 'ro', markersize=4, label=f'Threshold 0.45 ({len(peaks_orig)} steps)')
plt.axvline(40.0, color='blue', linestyle='--', label='t = 40s (Midpoint)')
plt.xlabel('Time (s)')
plt.ylabel('Vertical Acc (m/s2)')
plt.title('Step Peak Detection in Dataset 1')
plt.legend()
plt.grid(True)
plt.savefig(r"c:\New PJ\OFF-GRID SURVIVAL\docs\peaks_comparison_dataset1.png", dpi=150)
print("Saved docs/peaks_comparison_dataset1.png")
