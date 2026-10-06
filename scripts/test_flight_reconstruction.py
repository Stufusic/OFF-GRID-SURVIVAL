import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
d2 = r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"

def test_dataset(data_dir):
    folder = os.path.basename(data_dir)
    print("="*80)
    print(f"KIỂM TRA TÁI TẠO QUỸ ĐẠO BẰNG GYRO YAW VÀ PHÂN ĐOẠN VẾ THANG: {folder}")
    print("="*80)
    
    df_acc = pd.read_csv(os.path.join(data_dir, "TotalAcceleration.csv"))
    df_orient = pd.read_csv(os.path.join(data_dir, "Orientation.csv"))
    df_gyro = pd.read_csv(os.path.join(data_dir, "Gyroscope.csv"))
    df_compass = pd.read_csv(os.path.join(data_dir, "Compass.csv"))
    
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
    
    # 1. Heading: Integrate gyro_z with initial heading from compass median of first 3s
    gz = df_gyro['z'].values
    t_gyro = df_gyro['t'].values
    dt = np.diff(t_gyro)
    
    init_bearing = np.median(df_compass[df_compass['t'] < 3.0]['magneticBearing'])
    
    yaw_rel = np.zeros(len(t_gyro))
    yaw_rel[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)
    
    # Step headings
    step_yaw_rel = np.interp(step_times, t_gyro, yaw_rel)
    step_headings = (init_bearing + step_yaw_rel) % 360
    
    # Detect turn / landing by rate of relative yaw change
    # or diff of step_yaw_rel
    dyaw_steps = np.abs(np.diff(step_yaw_rel))
    dyaw_steps = np.insert(dyaw_steps, 0, 0)
    
    # Moving window turn detector (e.g. if turned > 40 deg in last 2 steps)
    is_landing = np.zeros(n_steps, dtype=bool)
    for i in range(n_steps):
        # check window of +/- 1 step
        i_start = max(0, i - 1)
        i_end = min(n_steps - 1, i + 1)
        total_turn = abs(step_yaw_rel[i_end] - step_yaw_rel[i_start])
        if total_turn > 45.0 or dyaw_steps[i] > 35.0:
            is_landing[i] = True
            
    print(f"Detected {np.sum(is_landing)} landing steps out of {n_steps} total steps")
    
    # Check top turnaround:
    # At top, cumulative heading reversal or time midpoint
    # Let's inspect step_yaw_rel over time to find where the turnaround is!
    # In Dataset 1, max / min yaw or reversal:
    print(f"Step yaw min: {np.min(step_yaw_rel):.1f}°, max: {np.max(step_yaw_rel):.1f}°")
    
    return step_times, step_headings, step_yaw_rel, is_landing

test_dataset(d1)
test_dataset(d2)
