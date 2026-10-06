import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"

df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))
df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))
df_compass = pd.read_csv(os.path.join(d1, "Compass.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0

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
n_steps = len(peaks)
half_steps = n_steps // 2

# Path with Orientation yaw
# In Android: yaw = 0 is North, pi/2 is West or East?
# Let's check Orientation yaw vs Compass
yaw_rad = np.interp(step_times, df_orient['t'], df_orient['yaw'])

# Let's check how yaw corresponds to compass
compass_at_steps = np.interp(step_times, df_compass['t'], df_compass['magneticBearing'])

# In ENU: x = East, y = North
# Compass bearing: 0 = North, 90 = East, 180 = South, 270 = West
# dx = L * sin(bearing), dy = L * cos(bearing)

# Let's test Orientation yaw: In Android Rotation Vector:
# Yaw is angle between device Y and North or device Y and Magnetic North?
# Let's compute trajectory using Orientation yaw:
w_tread = 0.27
xo, yo = [0.0], [0.0]
for y in yaw_rad:
    # Android yaw is typically: 0 = North, counter-clockwise positive (West = +pi/2, South = pi, East = -pi/2)
    # So bearing = -yaw
    b_rad = -y
    xo.append(xo[-1] + w_tread * np.sin(b_rad))
    yo.append(yo[-1] + w_tread * np.cos(b_rad))

xo = np.array(xo)
yo = np.array(yo)
net_o = np.sqrt(xo[-1]**2 + yo[-1]**2)

print(f"Orientation Yaw net 2D drift: {net_o:.2f} m")
print(f"Orientation Yaw bounding box: X span = {xo.max() - xo.min():.2f}m, Y span = {yo.max() - yo.min():.2f}m")

plt.figure(figsize=(7, 7))
plt.plot(xo[:half_steps+1], yo[:half_steps+1], 'b.-', label='Leo lên')
plt.plot(xo[half_steps:], yo[half_steps:], 'r.-', label='Đi xuống')
plt.plot([xo[0]], [yo[0]], 'go', markersize=10, label='Bắt đầu')
plt.plot([xo[-1]], [yo[-1]], 'ks', markersize=10, label=f'Kết thúc (cách {net_o:.2f}m)')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title(f'Quỹ đạo 2D bằng Orientation Yaw (Hộp: {xo.max()-xo.min():.1f}m x {yo.max()-yo.min():.1f}m)')
plt.savefig(r"c:\New PJ\OFF-GRID SURVIVAL\docs\test_orientation_yaw_2d.png", dpi=150)
print("Saved docs/test_orientation_yaw_2d.png")
