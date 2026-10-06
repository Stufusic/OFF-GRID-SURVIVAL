import os
import pandas as pd
import numpy as np
from scipy import signal

for path in [r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1", r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"]:
    folder = os.path.basename(path)
    print("\n" + "="*80)
    print(f"DEEP KINEMATIC FEATURE ANALYSIS: {folder}")
    print("="*80)
    
    df_acc = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
    df_att = pd.read_csv(os.path.join(path, "Orientation.csv"))
    df_gyro = pd.read_csv(os.path.join(path, "Gyroscope.csv"))
    df_compass = pd.read_csv(os.path.join(path, "Compass.csv"))
    
    t0 = df_acc['seconds_elapsed'].iloc[0]
    df_acc['t'] = df_acc['seconds_elapsed'] - t0
    df_att['t'] = df_att['seconds_elapsed'] - t0
    df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
    df_compass['t'] = df_compass['seconds_elapsed'] - t0
    
    duration = df_acc['t'].iloc[-1]
    fs = len(df_acc) / duration
    dt = 1.0 / fs
    
    # Attitude
    qw = np.interp(df_acc['t'], df_att['t'], df_att['qw'])
    qx = np.interp(df_acc['t'], df_att['t'], df_att['qx'])
    qy = np.interp(df_acc['t'], df_att['t'], df_att['qy'])
    qz = np.interp(df_acc['t'], df_att['t'], df_att['qz'])
    qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm
    
    # Gravity in body frame
    gx_b = 2.0 * (qx*qz - qw*qy)
    gy_b = 2.0 * (qy*qz + qw*qx)
    gz_b = 1.0 - 2.0 * (qx*qx + qy*qy)
    
    ax = df_acc['x'].values
    ay = df_acc['y'].values
    az = df_acc['z'].values
    
    # Forward body axis: perpendicular to gravity, pointing along phone top (y body)
    # Pitch angular velocity: gyro x
    gx_gyro = np.interp(df_acc['t'], df_gyro['t'], df_gyro['x'])
    gy_gyro = np.interp(df_acc['t'], df_gyro['t'], df_gyro['y'])
    gz_gyro = np.interp(df_acc['t'], df_gyro['t'], df_gyro['z'])
    
    a_v_dyn = (ax * gx_b + ay * gy_b + az * gz_b) - 9.80665
    
    # Step detection
    nyq = 0.5 * fs
    b, a = signal.butter(2, [1.0 / nyq, 3.2 / nyq], btype='band')
    az_filt = signal.filtfilt(b, a, a_v_dyn)
    peaks, _ = signal.find_peaks(az_filt, height=0.45, distance=int(0.28 * fs))
    step_times = df_acc['t'].iloc[peaks].values
    
    t_mid = duration / 2.0
    
    # Compute per-step features
    features = []
    for k in range(len(peaks)):
        p = peaks[k]
        t = step_times[k]
        
        # Step interval
        p_prev = peaks[k-1] if k > 0 else max(0, p - int(0.5 * fs))
        step_dt = (p - p_prev) * dt
        
        # Gyro pitch rate mean in this step
        mean_gx = np.mean(gx_gyro[p_prev:p+1])
        # Body forward accel mean
        mean_ay = np.mean(ay[p_prev:p+1])
        # Accel norm variance
        norm_sub = np.sqrt(ax[p_prev:p+1]**2 + ay[p_prev:p+1]**2 + az[p_prev:p+1]**2)
        var_norm = np.var(norm_sub)
        # Vertical jerk
        jerk_val = (az_filt[p] - az_filt[max(0, p-5)]) / (5 * dt)
        
        is_first_half = (t < t_mid)
        features.append({
            'k': k + 1,
            't': t,
            'is_ascent_true': is_first_half,
            'step_period': step_dt,
            'mean_gx': mean_gx,
            'mean_ay': mean_ay,
            'var_norm': var_norm,
            'jerk': jerk_val
        })
        
    df_feat = pd.DataFrame(features)
    asc = df_feat[df_feat['is_ascent_true']]
    desc = df_feat[~df_feat['is_ascent_true']]
    
    print("Feature Comparison (Mean +/- Std):")
    print(f"1. Step Period (s):")
    print(f"   Ascent : {asc['step_period'].mean():.3f} +/- {asc['step_period'].std():.3f}")
    print(f"   Descent: {desc['step_period'].mean():.3f} +/- {desc['step_period'].std():.3f}")
    
    print(f"2. Mean Gyro Pitch Rate Gx (rad/s):")
    print(f"   Ascent : {asc['mean_gx'].mean():.3f} +/- {asc['mean_gx'].std():.3f}")
    print(f"   Descent: {desc['mean_gx'].mean():.3f} +/- {desc['mean_gx'].std():.3f}")
    
    print(f"3. Mean Body Forward Accel Ay (m/s^2):")
    print(f"   Ascent : {asc['mean_ay'].mean():.3f} +/- {asc['mean_ay'].std():.3f}")
    print(f"   Descent: {desc['mean_ay'].mean():.3f} +/- {desc['mean_ay'].std():.3f}")
    
    print(f"4. Total Accel Variance:")
    print(f"   Ascent : {asc['var_norm'].mean():.3f} +/- {asc['var_norm'].std():.3f}")
    print(f"   Descent: {desc['var_norm'].mean():.3f} +/- {desc['var_norm'].std():.3f}")
