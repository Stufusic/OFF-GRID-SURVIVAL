import os
import pandas as pd
import numpy as np
from scipy import signal

path = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_acc = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
df_att = pd.read_csv(os.path.join(path, "Orientation.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_att['t'] = df_att['seconds_elapsed'] - t0

# Quaternion projection
qw = np.interp(df_acc['t'], df_att['t'], df_att['qw'])
qx = np.interp(df_acc['t'], df_att['t'], df_att['qx'])
qy = np.interp(df_acc['t'], df_att['t'], df_att['qy'])
qz = np.interp(df_acc['t'], df_att['t'], df_att['qz'])
qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm

gx_b = 2.0 * (qx*qz - qw*qy)
gy_b = 2.0 * (qy*qz + qw*qx)
gz_b = 1.0 - 2.0 * (qx*qx + qy*qy)

ax = df_acc['x'].values
ay = df_acc['y'].values
az = df_acc['z'].values

a_v_dyn = (ax * gx_b + ay * gy_b + az * gz_b) - 9.80665
df_acc['a_v'] = a_v_dyn

fs = len(df_acc) / df_acc['t'].iloc[-1]
nyq = 0.5 * fs
b, a = signal.butter(2, [1.0 / nyq, 3.2 / nyq], btype='band')
a_v_filt = signal.filtfilt(b, a, a_v_dyn)

peaks, _ = signal.find_peaks(a_v_filt, height=0.45, distance=int(0.28 * fs))
print(f"Total detected peaks: {len(peaks)}")

dt = 1.0 / fs
step_records = []
for k in range(len(peaks)):
    p = peaks[k]
    t = df_acc['t'].iloc[p]
    
    # Analyze the window around this step:
    # Look at 0.4s before the peak
    win_start = max(0, p - int(0.40 * fs))
    # Trough: minimum a_v in the 0.35s preceding the peak
    search_trough = max(0, p - int(0.35 * fs))
    trough_sub = np.argmin(a_v_dyn[search_trough:p+1])
    trough_idx = search_trough + trough_sub
    
    trough_val = a_v_dyn[trough_idx]
    peak_val = a_v_dyn[p]
    p2p = peak_val - trough_val
    rise_time = (p - trough_idx) * dt
    
    # Freefall in the step window (e.g. 0.35s before peak):
    step_win = a_v_dyn[search_trough:p+1]
    freefall_ms = np.sum(step_win < -2.0) * dt * 1000.0
    
    # Delta z from double integration in the step window with zero velocity start
    vz = np.cumsum(step_win) * dt
    dz = np.sum(vz) * dt
    
    # Waveform skewness
    half = len(step_win) // 2
    e1 = np.sum(step_win[:half]**2)
    e2 = np.sum(step_win[half:]**2)
    skew = (e1 - e2) / (e1 + e2 + 1e-6)
    
    step_records.append({
        'k': k + 1,
        't': round(t, 2),
        'rise_time': round(rise_time, 3),
        'freefall_ms': round(freefall_ms, 1),
        'p2p': round(p2p, 2),
        'dz': round(dz, 3),
        'skew': round(skew, 2),
        'trough': round(trough_val, 2),
        'peak': round(peak_val, 2)
    })

df_steps = pd.DataFrame(step_records)
print("\n--- FIRST HALF (ASCENT: t < 40s) ---")
print(df_steps[df_steps['t'] < 40][['k', 't', 'rise_time', 'freefall_ms', 'p2p', 'dz', 'skew']].describe())

print("\n--- SECOND HALF (DESCENT: t >= 40s) ---")
print(df_steps[df_steps['t'] >= 40][['k', 't', 'rise_time', 'freefall_ms', 'p2p', 'dz', 'skew']].describe())
