import os
import pandas as pd
import numpy as np
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
d2 = r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"

for d in [d1, d2]:
    folder = os.path.basename(d)
    print("="*60)
    print("DATASET:", folder)
    
    df_acc = pd.read_csv(os.path.join(d, "TotalAcceleration.csv"))
    df_orient = pd.read_csv(os.path.join(d, "Orientation.csv"))
    df_gyro = pd.read_csv(os.path.join(d, "Gyroscope.csv"))
    
    t0 = df_acc['seconds_elapsed'].iloc[0]
    df_acc['t'] = df_acc['seconds_elapsed'] - t0
    df_orient['t'] = df_orient['seconds_elapsed'] - t0
    df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
    
    fs = len(df_acc) / df_acc['t'].iloc[-1]
    
    # Step detection
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
    
    gz = df_gyro['z'].values
    t_gyro = df_gyro['t'].values
    dt = np.diff(t_gyro)
    yaw_rel = np.zeros(len(t_gyro))
    yaw_rel[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)
    step_yaw = np.interp(step_times, t_gyro, yaw_rel)
    
    # Delta yaw between consecutive steps
    dyaw = np.diff(step_yaw)
    print(f"Total steps: {len(peaks)}")
    # Find all major turns (|cumulative turn over 3 steps| > 50 deg)
    turns = []
    i = 0
    while i < len(step_yaw) - 3:
        delta = step_yaw[i+3] - step_yaw[i]
        if abs(delta) > 50:
            turns.append((i, step_times[i], delta))
            i += 3 # skip turn window
        else:
            i += 1
            
    print(f"Detected {len(turns)} major turns:")
    for step_idx, t_val, turn_deg in turns:
        direction = "QUAY TRÁI (CCW +)" if turn_deg > 0 else "QUAY PHẢI (CW -)"
        print(f"  Step {step_idx:3d} (t={t_val:4.1f}s): {turn_deg:+6.1f}° -> {direction}")
