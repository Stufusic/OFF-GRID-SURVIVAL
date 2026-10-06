import os
import pandas as pd
import numpy as np
from scipy import signal

for path in [r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1", r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"]:
    folder = os.path.basename(path)
    print("="*70)
    print(f"ANALYZING DISCRIMINATIVE SIGNALS FOR: {folder}")
    print("="*70)
    
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
    t_mid = duration / 2.0
    
    # Split into known Ascent phase (first half) and Descent phase (second half)
    asc_acc = df_acc[df_acc['t'] < t_mid]
    desc_acc = df_acc[df_acc['t'] >= t_mid]
    
    asc_gyro = df_gyro[df_gyro['t'] < t_mid]
    desc_gyro = df_gyro[df_gyro['t'] >= t_mid]
    
    asc_att = df_att[df_att['t'] < t_mid]
    desc_att = df_att[df_att['t'] >= t_mid]
    
    # 1. Linear acceleration (x, y, z body)
    print("1. BODY ACCELERATION:")
    print(f"   Ascent  ax={asc_acc['x'].mean():.2f}, ay={asc_acc['y'].mean():.2f}, az={asc_acc['z'].mean():.2f}")
    print(f"   Descent ax={desc_acc['x'].mean():.2f}, ay={desc_acc['y'].mean():.2f}, az={desc_acc['z'].mean():.2f}")
    
    # 2. Gyroscope means and variances
    print("2. GYROSCOPE (rad/s):")
    print(f"   Ascent  gx={asc_gyro['x'].mean():.3f} (std={asc_gyro['x'].std():.3f}), gy={asc_gyro['y'].mean():.3f} (std={asc_gyro['y'].std():.3f}), gz={asc_gyro['z'].mean():.3f} (std={asc_gyro['z'].std():.3f})")
    print(f"   Descent gx={desc_gyro['x'].mean():.3f} (std={desc_gyro['x'].std():.3f}), gy={desc_gyro['y'].mean():.3f} (std={desc_gyro['y'].std():.3f}), gz={desc_gyro['z'].mean():.3f} (std={desc_gyro['z'].std():.3f})")
    
    # 3. Attitude Pitch & Roll
    print("3. ATTITUDE (deg):")
    print(f"   Ascent  pitch={np.degrees(asc_att['pitch'].mean()):.1f} (std={np.degrees(asc_att['pitch'].std()):.1f}), roll={np.degrees(asc_att['roll'].mean()):.1f} (std={np.degrees(asc_att['roll'].std()):.1f})")
    print(f"   Descent pitch={np.degrees(desc_att['pitch'].mean()):.1f} (std={np.degrees(desc_att['pitch'].std()):.1f}), roll={np.degrees(desc_att['roll'].mean()):.1f} (std={np.degrees(desc_att['roll'].std()):.1f})")
    
    # 4. Turn analysis: look at yaw rate integrated
    yaw_rate = df_gyro['z'].values
    dt_gyro = np.diff(df_gyro['t'].values)
    yaw_cum = np.cumsum(yaw_rate[:-1] * dt_gyro)
    print(f"4. TOTAL YAW ROTATION CUMULATIVE: {np.degrees(yaw_cum[-1]):.1f} deg")
