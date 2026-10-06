import os
import pandas as pd

d1 = r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"
d2 = r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"

for d in [d1, d2]:
    print("="*50)
    print(os.path.basename(d))
    meta = pd.read_csv(os.path.join(d, "Metadata.csv"))
    print(meta.to_dict(orient='records'))
