import os
import pandas as pd
import numpy as np

path = r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"
df_acc = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
df_att = pd.read_csv(os.path.join(path, "Orientation.csv"))
df_compass = pd.read_csv(os.path.join(path, "Compass.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_att['t'] = df_att['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0

total_t = df_acc['t'].iloc[-1]
print(f"Dataset 2 duration: {total_t:.2f}s")

# Compass bearing in 5s windows
for t in range(0, int(total_t), 4):
    sub = df_compass[(df_compass['t'] >= t) & (df_compass['t'] < t + 4)]
    b_mean = sub['magneticBearing'].mean()
    b_std = sub['magneticBearing'].std()
    p_mean = df_att[(df_att['t'] >= t) & (df_att['t'] < t + 4)]['pitch'].mean()
    print(f"t={t:2d}s - {t+4:2d}s: Bearing={b_mean:5.1f} deg (std={b_std:4.1f}), Pitch={np.degrees(p_mean):5.1f} deg")
