import os
import pandas as pd
import numpy as np

path = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_acc = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
df_att = pd.read_csv(os.path.join(path, "Orientation.csv"))
df_compass = pd.read_csv(os.path.join(path, "Compass.csv"))

t0 = df_acc['seconds_elapsed'].iloc[0]
t_end = df_acc['seconds_elapsed'].iloc[-1]
t_mid = t0 + (t_end - t0) / 2.0

print(f"Dataset 1 duration: {t_end - t0:.2f}s, midpoint: {t_mid - t0:.2f}s")

half1_att = df_att[df_att['seconds_elapsed'] < t_mid]
half2_att = df_att[df_att['seconds_elapsed'] >= t_mid]

print(f"Half 1 (0 to {t_mid - t0:.1f}s):")
print(f"  pitch mean: {half1_att['pitch'].mean():.3f}, std: {half1_att['pitch'].std():.3f}")
print(f"  roll mean : {half1_att['roll'].mean():.3f}, std: {half1_att['roll'].std():.3f}")
print(f"  yaw mean  : {half1_att['yaw'].mean():.3f}, std: {half1_att['yaw'].std():.3f}")

print(f"Half 2 ({t_mid - t0:.1f}s to {t_end - t0:.1f}s):")
print(f"  pitch mean: {half2_att['pitch'].mean():.3f}, std: {half2_att['pitch'].std():.3f}")
print(f"  roll mean : {half2_att['roll'].mean():.3f}, std: {half2_att['roll'].std():.3f}")
print(f"  yaw mean  : {half2_att['yaw'].mean():.3f}, std: {half2_att['yaw'].std():.3f}")

half1_comp = df_compass[df_compass['seconds_elapsed'] < t_mid]
half2_comp = df_compass[df_compass['seconds_elapsed'] >= t_mid]
print(f"Half 1 compass bearing: mean={half1_comp['magneticBearing'].mean():.1f}, std={half1_comp['magneticBearing'].std():.1f}")
print(f"Half 2 compass bearing: mean={half2_comp['magneticBearing'].mean():.1f}, std={half2_comp['magneticBearing'].std():.1f}")
