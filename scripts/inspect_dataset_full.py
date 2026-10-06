import os
import glob
import pandas as pd
import numpy as np

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
d2 = r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"

for d in [d1, d2]:
    print("="*60)
    print("DATASET:", os.path.basename(d))
    files = glob.glob(os.path.join(d, "*.csv"))
    for f in sorted(files):
        sz = os.path.getsize(f)
        try:
            df = pd.read_csv(f, nrows=2)
            cols = list(df.columns)
            print(f"  {os.path.basename(f):30s} {sz/1024:8.1f} KB  cols: {cols}")
        except Exception as e:
            print(f"  {os.path.basename(f):30s} {sz/1024:8.1f} KB  err: {e}")
