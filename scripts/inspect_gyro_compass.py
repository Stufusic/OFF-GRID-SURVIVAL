import os
import pandas as pd
import numpy as np

path = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_acc = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
df_att = pd.read_csv(os.path.join(path, "Orientation.csv"))
df_gyro = pd.read_csv(os.path.join(path, "Gyroscope.csv"))
df_compass = pd.read_csv(os.path.join(path, "Compass.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
df_acc['t'] = df_acc['seconds_elapsed'] - t0
df_att['t'] = df_att['seconds_elapsed'] - t0
df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
df_compass['t'] = df_compass['seconds_elapsed'] - t0

h1_gyro = df_gyro[df_gyro['t'] < 40]
h2_gyro = df_gyro[df_gyro['t'] >= 40]

print("GYRO X (pitch rate):")
print("  Half 1 mean:", h1_gyro['x'].mean(), "std:", h1_gyro['x'].std())
print("  Half 2 mean:", h2_gyro['x'].mean(), "std:", h2_gyro['x'].std())

print("GYRO Y (roll rate):")
print("  Half 1 mean:", h1_gyro['y'].mean(), "std:", h1_gyro['y'].std())
print("  Half 2 mean:", h2_gyro['y'].mean(), "std:", h2_gyro['y'].std())

print("GYRO Z (yaw rate):")
print("  Half 1 mean:", h1_gyro['z'].mean(), "std:", h1_gyro['z'].std())
print("  Half 2 mean:", h2_gyro['z'].mean(), "std:", h2_gyro['z'].std())

# Let's inspect the turn at t ~ 40s (the peak of the staircase where the user reaches top floor)
print("\nCompass and Orientation around turnaround (t = 35s to 45s):")
sub_comp = df_compass[(df_compass['t'] >= 35) & (df_compass['t'] <= 45)]
for idx, r in sub_comp.iterrows():
    print(f"t={r['t']:.2f}s: bearing={r['magneticBearing']:.1f}")
