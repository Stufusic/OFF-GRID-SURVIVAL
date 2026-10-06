import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"

df_orient = pd.read_csv(os.path.join(d1, "Orientation.csv"))
df_gyro = pd.read_csv(os.path.join(d1, "Gyroscope.csv"))
df_compass = pd.read_csv(os.path.join(d1, "Compass.csv"))
df_acc = pd.read_csv(os.path.join(d1, "TotalAcceleration.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
for df in [df_orient, df_gyro, df_compass, df_acc]:
    df['t'] = df['seconds_elapsed'] - t0

# Integrate gyro_z to get relative yaw angle
gz = df_gyro['z'].values
t_gyro = df_gyro['t'].values
dt = np.diff(t_gyro)
integrated_yaw_deg = np.zeros(len(t_gyro))
integrated_yaw_deg[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)

# Orientation yaw unwrapped in degrees
yaw_orient_deg = np.unwrap(df_orient['yaw'].values) * (180.0 / np.pi)
# Compass unwrapped
comp = df_compass['magneticBearing'].values
comp_diff = (np.diff(comp) + 180) % 360 - 180
comp_unwrap = np.zeros(len(comp))
comp_unwrap[0] = comp[0]
comp_unwrap[1:] = comp[0] + np.cumsum(comp_diff)

fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

axs[0].plot(df_compass['t'], comp_unwrap, label='Compass Unwrapped (deg)', color='orange')
axs[0].plot(df_orient['t'], yaw_orient_deg, label='Orientation Yaw Unwrapped (deg)', color='blue')
axs[0].plot(t_gyro, integrated_yaw_deg, label='Gyro Z Integrated (deg)', color='green', linestyle='--')
axs[0].set_ylabel('Heading / Yaw (deg)')
axs[0].legend()
axs[0].grid(True)
axs[0].set_title('Heading Comparison (Tracking Stairwell Turns)')

# Pitch from Orientation
axs[1].plot(df_orient['t'], np.degrees(df_orient['pitch']), label='Pitch (deg)', color='purple')
axs[1].plot(df_orient['t'], np.degrees(df_orient['roll']), label='Roll (deg)', color='gray', alpha=0.5)
axs[1].set_ylabel('Attitude (deg)')
axs[1].legend()
axs[1].grid(True)

# Gyro X (pitch rate) and Gyro Z (yaw rate)
axs[2].plot(df_gyro['t'], df_gyro['x'], label='Gyro X (pitch rate rad/s)', color='red', alpha=0.6)
axs[2].plot(df_gyro['t'], df_gyro['z'], label='Gyro Z (yaw rate rad/s)', color='teal', alpha=0.6)
axs[2].set_ylabel('Angular Vel (rad/s)')
axs[2].legend()
axs[2].grid(True)

# Acc norm
acc_mag = np.sqrt(df_acc['x']**2 + df_acc['y']**2 + df_acc['z']**2)
axs[3].plot(df_acc['t'], acc_mag, label='Total Acc Magnitude (m/s2)', color='black', lw=0.8)
axs[3].set_ylabel('Acc (m/s2)')
axs[3].set_xlabel('Time (seconds)')
axs[3].legend()
axs[3].grid(True)

plt.tight_layout()
plt.savefig(r"c:\New PJ\OFF-GRID SURVIVAL\docs\sensor_truth_analysis_1.png", dpi=150)
print("Saved sensor_truth_analysis_1.png")
