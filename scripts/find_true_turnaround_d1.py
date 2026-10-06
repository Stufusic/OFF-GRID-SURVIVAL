import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))
df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0

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
peaks, _ = signal.find_peaks(az_filt, height=0.40, distance=int(0.28 * fs))
step_times = df_acc['t'].iloc[peaks].values
n_steps = len(peaks)

yaw_rad = np.interp(step_times, df_orient['t'], df_orient['yaw'])
w_tread = 0.27
xo, yo = [0.0], [0.0]
for y in yaw_rad:
    b_rad = -y
    xo.append(xo[-1] + w_tread * np.sin(b_rad))
    yo.append(yo[-1] + w_tread * np.cos(b_rad))

xo = np.array(xo)
yo = np.array(yo)

dist_from_start = np.sqrt(xo**2 + yo**2)
max_dist_step = np.argmax(dist_from_start)

print(f"Dataset 1 Total steps: {n_steps}")
print(f"Max distance from start occurs at step {max_dist_step} (t = {step_times[max_dist_step-1]:.1f}s): dist = {dist_from_start[max_dist_step]:.2f}m")
print(f"Coordinate at max dist: ({xo[max_dist_step]:.2f}, {yo[max_dist_step]:.2f})")
print(f"End coordinate: ({xo[-1]:.2f}, {yo[-1]:.2f}) -> net error = {dist_from_start[-1]:.2f}m")

plt.figure(figsize=(8, 8))
plt.plot(xo[:max_dist_step+1], yo[:max_dist_step+1], 'b.-', label=f'Lượt Đi / Leo lên ({max_dist_step} bước)')
plt.plot(xo[max_dist_step:], yo[max_dist_step:], 'r.-', label=f'Lượt Về / Đi xuống ({n_steps - max_dist_step} bước)')
plt.plot([xo[0]], [yo[0]], 'go', markersize=12, label='BẮT ĐẦU (0, 0)')
plt.plot([xo[max_dist_step]], [yo[max_dist_step]], 'm^', markersize=12, label=f'ĐỈNH / ĐIỂM QUAY ĐẦU (Cách {dist_from_start[max_dist_step]:.2f}m)')
plt.plot([xo[-1]], [yo[-1]], 'ks', markersize=12, label=f'KẾT THÚC (Cách {dist_from_start[-1]:.2f}m)')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title(f'Dataset 1: Quỹ Đạo Thật Sự Với Điểm Quay Đầu Tự Động (Bước {max_dist_step})')
plt.savefig(r"c:\New PJ\OFF-GRID SURVIVAL\docs\dataset1_true_turnaround.png", dpi=150)
print("Saved docs/dataset1_true_turnaround.png")
