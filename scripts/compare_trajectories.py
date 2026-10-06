import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"

df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))
df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))
df_gyro = pd.read_csv(os.path.join(d1, "Gyroscope.csv"))
df_compass = pd.read_csv(os.path.join(d1, "Compass.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0

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

b, a = signal.butter(2, [1.0/(0.5*fs), 3.2/(0.5*fs)], btype='band')
az_filt = signal.filtfilt(b, a, az_w)
peaks, _ = signal.find_peaks(az_filt, height=0.45, distance=int(0.28 * fs))
step_times = df_acc['t'].iloc[peaks].values
n_steps = len(peaks)
half_steps = n_steps // 2

# Path with Compass
compass_steps = np.interp(step_times, df_compass['t'], df_compass['magneticBearing'])
w_tread = 0.27
xc, yc = [0.0], [0.0]
for b_deg in compass_steps:
    rad = np.radians(b_deg)
    xc.append(xc[-1] + w_tread * np.sin(rad))
    yc.append(yc[-1] + w_tread * np.cos(rad))

# Path with Gyro Yaw (relative angle + initial compass)
gz = df_gyro['z'].values
t_gyro = df_gyro['t'].values
dt = np.diff(t_gyro)
yaw_rel = np.zeros(len(t_gyro))
yaw_rel[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)
init_b = np.median(df_compass[df_compass['t'] < 3.0]['magneticBearing'])
gyro_heading = (init_b + np.interp(step_times, t_gyro, yaw_rel)) % 360

xg, yg = [0.0], [0.0]
for b_deg in gyro_heading:
    rad = np.radians(b_deg)
    xg.append(xg[-1] + w_tread * np.sin(rad))
    yg.append(yg[-1] + w_tread * np.cos(rad))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

ax1.plot(xc[:half_steps+1], yc[:half_steps+1], 'b.-', label=f'Leo lên ({half_steps} bước)')
ax1.plot(xc[half_steps:], yc[half_steps:], 'r.-', label=f'Đi xuống ({n_steps - half_steps} bước)')
ax1.plot([xc[0]], [yc[0]], 'go', markersize=10, label='Bắt đầu')
ax1.plot([xc[-1]], [yc[-1]], 'ks', markersize=10, label=f'Kết thúc (lệch {np.sqrt(xc[-1]**2 + yc[-1]**2):.2f}m)')
ax1.set_title('Dùng COMPASS gốc (Bị nhiễu từ trường buồng thang)\n-> Hai vế thang bị xô lệch 3 mét!')
ax1.set_xlabel('East (m)')
ax1.set_ylabel('North (m)')
ax1.axis('equal')
ax1.grid(True)
ax1.legend()

ax2.plot(xg[:half_steps+1], yg[:half_steps+1], 'b.-', label=f'Leo lên ({half_steps} bước)')
ax2.plot(xg[half_steps:], yg[half_steps:], 'r.-', label=f'Đi xuống ({n_steps - half_steps} bước)')
ax2.plot([xg[0]], [yg[0]], 'go', markersize=10, label='Bắt đầu')
ax2.plot([xg[-1]], [yg[-1]], 'ks', markersize=10, label=f'Kết thúc (lệch {np.sqrt(xg[-1]**2 + yg[-1]**2):.2f}m)')
ax2.set_title('Dùng GYROSCOPE Z (Miễn nhiễm hoàn toàn nhiễu từ)\n-> Quỹ đạo lên và xuống chập khít vào cùng buồng thang!')
ax2.set_xlabel('East (m)')
ax2.set_ylabel('North (m)')
ax2.axis('equal')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.savefig(r"c:\New PJ\OFF-GRID SURVIVAL\docs\compass_vs_gyro_heading_2d.png", dpi=150)
print("Saved docs/compass_vs_gyro_heading_2d.png")
