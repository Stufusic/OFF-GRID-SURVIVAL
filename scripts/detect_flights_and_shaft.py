import os
import pandas as pd
import numpy as np
from scipy import signal

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
d2 = r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"

def analyze_stair_structure(d):
    folder = os.path.basename(d)
    print("="*70)
    print(f"PHÂN TÍCH CẤU TRÚC VẾ THANG & CHIẾU NGHỈ: {folder}")
    print("="*70)
    
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
    n_steps = len(peaks)
    
    # Gyro Z integrated yaw (smooth, no magnetic distortion)
    gz = df_gyro['z'].values
    t_gyro = df_gyro['t'].values
    dt = np.diff(t_gyro)
    yaw_gyro = np.zeros(len(t_gyro))
    yaw_gyro[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)
    step_yaw_gyro = np.interp(step_times, t_gyro, yaw_gyro)
    
    # Check turns: rate of yaw change or delta yaw between steps
    dyaw = np.diff(step_yaw_gyro)
    
    # Segment into flights and landings based on gyro yaw
    # In a stairwell, each flight has roughly constant heading, and landings have ~180 deg turns
    print(f"Tổng số bước: {n_steps}")
    print(f"Yaw đầu: {step_yaw_gyro[0]:.1f}°, Yaw cuối: {step_yaw_gyro[-1]:.1f}°")
    print(f"Yaw min: {step_yaw_gyro.min():.1f}°, Yaw max: {step_yaw_gyro.max():.1f}°")
    
    # Let's print out segments
    segments = []
    curr_seg = [0]
    for i in range(1, n_steps):
        # If yaw difference from start of current segment is large (> 60 deg)
        if abs(step_yaw_gyro[i] - step_yaw_gyro[curr_seg[0]]) > 60:
            segments.append(curr_seg)
            curr_seg = [i]
        else:
            curr_seg.append(i)
    if curr_seg:
        segments.append(curr_seg)
        
    print(f"Số đoạn di chuyển (vế thang / chiếu nghỉ): {len(segments)}")
    for idx, seg in enumerate(segments):
        t_start = step_times[seg[0]]
        t_end = step_times[seg[-1]]
        yaw_mean = np.mean(step_yaw_gyro[seg])
        print(f"  Đoạn {idx+1:2d}: bước {seg[0]+1:3d} -> {seg[-1]+1:3d} ({len(seg):2d} bước) | t = {t_start:4.1f}s - {t_end:4.1f}s | Yaw TB = {yaw_mean:6.1f}°")

analyze_stair_structure(d1)
analyze_stair_structure(d2)
