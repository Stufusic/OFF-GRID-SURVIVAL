import os
import sys
import numpy as np
import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt

DATA_DIR = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
folder_name = os.path.basename(DATA_DIR.rstrip("\\/"))
OUT_DIR = r"c:\New PJ\OFF-GRID SURVIVAL\docs"
os.makedirs(OUT_DIR, exist_ok=True)

print(f"Loading data from {DATA_DIR}...")
files = os.listdir(DATA_DIR)
print(f"Files found: {files}")

# Read metadata if exists
meta_path = os.path.join(DATA_DIR, "Metadata.csv")
if os.path.exists(meta_path):
    with open(meta_path, 'r', encoding='utf-8') as fh:
        print("Metadata:\n" + fh.read().strip())

df_acc = pd.read_csv(os.path.join(DATA_DIR, "TotalAcceleration.csv"))
df_lin_acc = pd.read_csv(os.path.join(DATA_DIR, "Accelerometer.csv"))
df_gyro = pd.read_csv(os.path.join(DATA_DIR, "Gyroscope.csv"))
df_orient = pd.read_csv(os.path.join(DATA_DIR, "Orientation.csv"))
df_mag = pd.read_csv(os.path.join(DATA_DIR, "Magnetometer.csv"))
df_compass = pd.read_csv(os.path.join(DATA_DIR, "Compass.csv"))
df_gravity = pd.read_csv(os.path.join(DATA_DIR, "Gravity.csv"))

# Normalize time to start at 0
t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_lin_acc['t'] = df_lin_acc['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
df_orient['t'] = df_orient['seconds_elapsed'] - t0
df_mag['t'] = df_mag['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0
df_gravity['t'] = df_gravity['seconds_elapsed'] - t0

total_duration = df_acc['t'].iloc[-1]
print(f"\nTotal recording duration: {total_duration:.2f} seconds ({len(df_acc)} samples)")
dt_mean = np.mean(np.diff(df_acc['t']))
sample_rate = 1.0 / dt_mean if dt_mean > 0 else 100.0
print(f"Mean sampling interval: {dt_mean*1000:.2f} ms (~{sample_rate:.1f} Hz)")

# Acceleration Norms
df_acc['norm'] = np.sqrt(df_acc['x']**2 + df_acc['y']**2 + df_acc['z']**2)
df_lin_acc['norm'] = np.sqrt(df_lin_acc['x']**2 + df_lin_acc['y']**2 + df_lin_acc['z']**2)
df_gyro['norm'] = np.sqrt(df_gyro['x']**2 + df_gyro['y']**2 + df_gyro['z']**2)
df_mag['norm'] = np.sqrt(df_mag['x']**2 + df_mag['y']**2 + df_mag['z']**2)

print("\n--- Summary Statistics ---")
print(f"Total Accel Norm: Mean={df_acc['norm'].mean():.2f} m/s^2, Std={df_acc['norm'].std():.2f}, Min={df_acc['norm'].min():.2f}, Max={df_acc['norm'].max():.2f}")
print(f"Linear Accel Norm: Mean={df_lin_acc['norm'].mean():.2f} m/s^2, Std={df_lin_acc['norm'].std():.2f}, Max={df_lin_acc['norm'].max():.2f}")
print(f"Gyro Norm: Mean={df_gyro['norm'].mean():.2f} rad/s, Std={df_gyro['norm'].std():.2f}, Max={df_gyro['norm'].max():.2f}")
print(f"Mag Norm: Mean={df_mag['norm'].mean():.2f} uT, Std={df_mag['norm'].std():.2f}, Min={df_mag['norm'].min():.2f}, Max={df_mag['norm'].max():.2f}")

# Bandpass filter for step detection (0.8 Hz - 3.5 Hz)
nyq = 0.5 * sample_rate
b, a = signal.butter(2, [0.8 / nyq, 3.5 / nyq], btype='band')
acc_filtered = signal.filtfilt(b, a, df_acc['norm'] - df_acc['norm'].mean())

# Peak detection for steps
min_distance_samples = int(0.28 * sample_rate)  # at least 280 ms between steps
peaks, properties = signal.find_peaks(acc_filtered, height=0.55, distance=min_distance_samples)
step_times = df_acc['t'].iloc[peaks].values
step_count = len(peaks)
print(f"\nStep Detection: Found {step_count} steps over {total_duration:.1f} s")
cadence_overall = (step_count / total_duration) * 60.0 if total_duration > 0 else 0
print(f"Overall Cadence: {cadence_overall:.1f} steps/min")

# Spectral analysis (FFT)
acc_detrend = (df_acc['norm'] - df_acc['norm'].mean()).to_numpy()
f_welch, pxx_welch = signal.welch(acc_detrend, fs=sample_rate, nperseg=min(1024, len(acc_detrend)))
dom_freq = f_welch[np.argmax(pxx_welch)]
print(f"Dominant Motion Frequency: {dom_freq:.2f} Hz")

# Time-sliced activity breakdown
window_sec = 5.0
n_windows = int(np.floor(total_duration / window_sec))
window_activities = []

for i in range(n_windows):
    t_start = i * window_sec
    t_end = t_start + window_sec
    mask_acc = (df_acc['t'] >= t_start) & (df_acc['t'] < t_end)
    mask_lin = (df_lin_acc['t'] >= t_start) & (df_lin_acc['t'] < t_end)
    mask_gyro = (df_gyro['t'] >= t_start) & (df_gyro['t'] < t_end)
    sub_acc = df_acc.loc[mask_acc, 'norm']
    sub_lin = df_lin_acc.loc[mask_lin, 'norm']
    sub_gyro = df_gyro.loc[mask_gyro, 'norm']
    
    # Slice filtered acc
    sub_filt = acc_filtered[mask_acc]
    sub_peaks, _ = signal.find_peaks(sub_filt, height=0.55, distance=min_distance_samples)
    w_steps = len(sub_peaks)
    w_cadence = (w_steps / window_sec) * 60.0
    w_std = sub_lin.std() if len(sub_lin) > 0 else 0
    w_gyro_mean = sub_gyro.mean() if len(sub_gyro) > 0 else 0
    
    if w_steps == 0 and w_std < 0.35:
        act = "STATIONARY (Dung yen / Dat may)"
    elif w_cadence > 130:
        act = "RUNNING (Chay bo)"
    elif w_steps >= 2 or w_cadence >= 40:
        act = "WALKING (Di bo)"
    elif w_std > 0.8:
        act = "HANDLING / FIDGETING (Cam tay / Chuyen dong tay)"
    else:
        act = "SLOW MOTION / IDLE"
        
    window_activities.append({
        'start': t_start,
        'end': t_end,
        'steps': w_steps,
        'cadence': w_cadence,
        'acc_std': w_std,
        'gyro_mean': w_gyro_mean,
        'activity': act
    })

print("\n--- Activity Timeline (5-second intervals) ---")
for w in window_activities:
    print(f"[{w['start']:5.1f}s - {w['end']:5.1f}s]: Steps={w['steps']:2d}, Cadence={w['cadence']:5.1f} spm, LinAccStd={w['acc_std']:.2f}, Activity: {w['activity']}")

# Orientation & Device Pose Analysis
mean_pitch = np.degrees(df_orient['pitch'].mean())
mean_roll = np.degrees(df_orient['roll'].mean())
mean_yaw = np.degrees(df_orient['yaw'].mean())
print(f"\nDevice Pose (Orientation): Mean Pitch={mean_pitch:.1f} deg, Mean Roll={mean_roll:.1f} deg")

# Gravity vector check (phone vertical tilt)
print(f"Gravity Vector: Mean Gx={df_gravity['x'].mean():.2f}, Gy={df_gravity['y'].mean():.2f}, Gz={df_gravity['z'].mean():.2f}")

# PDR Trajectory Reconstruction
compass_t = df_compass['t'].values
compass_bearing = df_compass['magneticBearing'].values

step_bearings = np.interp(step_times, compass_t, compass_bearing) if len(step_times) > 0 else []

x_pos = [0.0]
y_pos = [0.0]
for idx, p_idx in enumerate(peaks):
    win_l = max(0, p_idx - int(0.15 * sample_rate))
    win_r = min(len(df_acc) - 1, p_idx + int(0.15 * sample_rate))
    a_max = df_acc['norm'].iloc[win_l:win_r].max()
    a_min = df_acc['norm'].iloc[win_l:win_r].min()
    diff = max(0.01, a_max - a_min)
    step_len = 0.45 * (diff ** 0.25)
    step_len = np.clip(step_len, 0.45, 0.85)
    
    heading_rad = np.radians(step_bearings[idx])
    dx = step_len * np.sin(heading_rad)
    dy = step_len * np.cos(heading_rad)
    x_pos.append(x_pos[-1] + dx)
    y_pos.append(y_pos[-1] + dy)

total_dist = sum(np.sqrt(np.diff(x_pos)**2 + np.diff(y_pos)**2)) if len(x_pos) > 1 else 0
net_displacement = np.sqrt(x_pos[-1]**2 + y_pos[-1]**2) if len(x_pos) > 1 else 0
print(f"\n--- Trajectory Estimation (PDR) ---")
print(f"Total Trajectory Distance: {total_dist:.2f} meters")
print(f"Net End-to-End Displacement: {net_displacement:.2f} meters")
if len(compass_bearing) > 0:
    print(f"Initial Heading: {compass_bearing[0]:.1f} deg, Final Heading: {compass_bearing[-1]:.1f} deg")

# Plotting summary visualization
fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=False)

# 1. Acceleration & Step detection
axs[0].plot(df_acc['t'], df_acc['norm'], label='Raw Accel Norm', color='gray', alpha=0.5)
axs[0].plot(df_acc['t'], acc_filtered + df_acc['norm'].mean(), label='Filtered Accel (1-3Hz)', color='blue', lw=1.5)
if len(peaks) > 0:
    axs[0].plot(df_acc['t'].iloc[peaks], df_acc['norm'].iloc[peaks], 'ro', markersize=4, label=f'Detected Steps ({step_count})')
axs[0].set_ylabel('Acceleration (m/s²)')
axs[0].set_title(f'1. Acceleration & Detected Steps ({folder_name})')
axs[0].legend(loc='upper right')
axs[0].grid(True, alpha=0.3)

# 2. Gyroscope Angular Rate
axs[1].plot(df_gyro['t'], df_gyro['x'], label='wx (Roll rate)', color='red', alpha=0.6)
axs[1].plot(df_gyro['t'], df_gyro['y'], label='wy (Pitch rate)', color='green', alpha=0.6)
axs[1].plot(df_gyro['t'], df_gyro['z'], label='wz (Yaw rate)', color='blue', alpha=0.6)
axs[1].plot(df_gyro['t'], df_gyro['norm'], label='Total Norm', color='black', lw=1.2)
axs[1].set_ylabel('Angular Vel (rad/s)')
axs[1].set_title('2. Gyroscope & Phone Angular Dynamics')
axs[1].legend(loc='upper right')
axs[1].grid(True, alpha=0.3)

# 3. Compass Heading
axs[2].plot(df_compass['t'], df_compass['magneticBearing'], label='Compass Heading', color='purple', lw=1.5)
axs[2].set_ylabel('Heading (degrees)')
axs[2].set_title('3. Magnetic Heading Over Time')
axs[2].set_ylim(0, 360)
axs[2].grid(True, alpha=0.3)
axs[2].legend(loc='upper right')

# 4. PDR Trajectory 2D
axs[3].plot(x_pos, y_pos, 'b.-', lw=1.5, markersize=4, label=f'Estimated Path ({total_dist:.1f}m)')
axs[3].plot(x_pos[0], y_pos[0], 'go', markersize=10, label='START')
axs[3].plot(x_pos[-1], y_pos[-1], 'rs', markersize=10, label='END')
axs[3].set_xlabel('East (meters)')
axs[3].set_ylabel('North (meters)')
axs[3].set_title('4. Reconstructed 2D Trajectory (Pedestrian Dead-Reckoning)')
axs[3].axis('equal')
axs[3].grid(True, alpha=0.3)
axs[3].legend(loc='best')

plt.tight_layout()
plot_path = os.path.join(OUT_DIR, f"real_dataset_prediction_{folder_name}.png")
plt.savefig(plot_path, dpi=150)
print(f"\nVisualization saved to: {plot_path}")
