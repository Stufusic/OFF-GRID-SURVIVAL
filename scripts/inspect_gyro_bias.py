import os
import pandas as pd
import numpy as np

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
df_u = pd.read_csv(os.path.join(d1, "GyroscopeUncalibrated.csv"), nrows=5)
print("GyroscopeUncalibrated columns:", df_u.columns)
print(df_u)

df_u_all = pd.read_csv(os.path.join(d1, "GyroscopeUncalibrated.csv"))
print("Shape:", df_u_all.shape)
if 'z_bias' in df_u_all.columns:
    print("z_bias mean:", df_u_all['z_bias'].mean())
