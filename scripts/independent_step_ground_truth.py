"""Ground truth độc lập cho số bước: 2 phương pháp peak-detection chuẩn trên
gia tốc thô (TotalAcceleration), không dùng thư viện dự án.

Method A: bandpass 0.8-3.5 Hz trên norm, find_peaks(distance=0.35s, prominence=1.2)
Method B: bandpass 1.0-3.0 Hz trên norm detrended, find_peaks(distance=0.40s, prominence=1.5)
Báo cáo số bước mỗi phương pháp + consensus.
"""
import os
import numpy as np
import pandas as pd
from scipy import signal

DATASETS = [
    r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1",
    r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1",
]

for data_dir in DATASETS:
    df = pd.read_csv(os.path.join(data_dir, "TotalAcceleration.csv"))
    fs = 1.0 / np.mean(np.diff(df["seconds_elapsed"]))
    norm = np.sqrt(df["x"]**2 + df["y"]**2 + df["z"]**2).values
    t = df["seconds_elapsed"].values - df["seconds_elapsed"].iloc[0]

    # Method A
    b, a = signal.butter(4, [0.8/(fs/2), 3.5/(fs/2)], btype="band")
    fA = signal.filtfilt(b, a, norm)
    peaksA, _ = signal.find_peaks(fA, distance=int(0.35*fs), prominence=1.2)

    # Method B
    b2, a2 = signal.butter(4, [1.0/(fs/2), 3.0/(fs/2)], btype="band")
    fB = signal.filtfilt(b2, a2, norm)
    peaksB, _ = signal.find_peaks(fB, distance=int(0.40*fs), prominence=1.5)

    cadA = len(peaksA) / t[-1]
    cadB = len(peaksB) / t[-1]
    print(f"{os.path.basename(data_dir)}: dur={t[-1]:.1f}s fs={fs:.0f}Hz")
    print(f"  Method A: {len(peaksA)} bước  (cadence {cadA:.2f} bước/s)")
    print(f"  Method B: {len(peaksB)} bước  (cadence {cadB:.2f} bước/s)")
    # Consensus: peaks khớp nhau trong cửa sổ 0.25s
    matched = 0
    for p in peaksA:
        if len(peaksB) and np.min(np.abs(peaksB/fs - p/fs)) <= 0.25:
            matched += 1
    print(f"  Consensus (peak A khớp peak B ±0.25s): {matched} bước")
    print()
