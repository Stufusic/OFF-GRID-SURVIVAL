"""Chạy replay_evaluation.exe trên toàn bộ recordings MotionSense đã convert,
phân tích kết quả và chấm độ chính xác theo nhãn hoạt động thật của dataset.

Ground truth của MotionSense là NHÃN HOẠT ĐỘNG do con người gán cho từng trial
(dws=downstairs, ups=upstairs, wlk=walking, jog=jogging, sit/std=stationary).
Số bước không có truth chính thức -> dùng peak-detection độc lập làm tham chiếu.
"""
import os
import re
import subprocess
import numpy as np
import pandas as pd
from scipy import signal

ROOT = r"C:\New PJ\OFF-GRID SURVIVAL\testdata\motionsense"
EXE = r"C:\New PJ\OFF-GRID SURVIVAL\build\replay_evaluation.exe"
G = 9.80665

ACT_ALIAS = {
    "STATIONARY": "STATIONARY",
    "WALKING_FLAT": "WALKING",
    "RUNNING": "RUNNING",
    "STAIRS_ASCENT": "STAIRS_UP",
    "STAIRS_DESCENT": "STAIRS_DOWN",
    "STAIR_LANDING": "STAIRS_LANDING",
    "ESCALATOR": "OTHER",
    "ELEVATOR": "OTHER",
    "CYCLING": "OTHER",
    "VEHICLE_WHEELED": "OTHER",
    "CABLE_TRANSPORT": "OTHER",
    "UNKNOWN": "OTHER",
}

def ref_step_count(path: str) -> int:
    df = pd.read_csv(os.path.join(path, "TotalAcceleration.csv"))
    fs = 50.0
    norm = np.sqrt(df["x"]**2 + df["y"]**2 + df["z"]**2).values
    if len(norm) < fs:
        return 0
    b, a = signal.butter(4, [0.7/(fs/2), 4.0/(fs/2)], btype="band")
    f = signal.filtfilt(b, a, norm)
    peaks, _ = signal.find_peaks(f, distance=int(0.35*fs), prominence=2.0)
    return len(peaks)

def parse_summary(out: str) -> dict:
    res = {"steps": 0, "loop_closure_m": np.nan, "segments": {}}
    m = re.search(r"2\. Tổng số bước nhận diện:\s*(\d+)", out)
    if m:
        res["steps"] = int(m.group(1))
    m = re.search(r"4\. Độ lệch khép kín vòng lặp \(Loop closure error\):\s*([\d.]+)", out)
    if m:
        res["loop_closure_m"] = float(m.group(1))
    m = re.search(r"1\. Tổng thời gian:\s*([\d.]+)", out)
    if m:
        res["duration_s"] = float(m.group(1))
    # Bảng segment: hoạt động bắt-đầu kết-thúc thời-lượng số-bước tốc-độ
    in_table = False
    for line in out.splitlines():
        if "Hoạt động" in line and "Bắt đầu" in line:
            in_table = True
            continue
        if in_table:
            mm = re.match(r"([A-Z_]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+(\d+)\s+([\d.]+)\s*$", line.strip())
            if mm:
                act = mm.group(1)
                dur = float(mm.group(4))
                res["segments"][act] = res["segments"].get(act, 0.0) + dur
            elif line.startswith("==="):
                in_table = False
    return res

def main():
    manifest = pd.read_csv(os.path.join(ROOT, "manifest.csv"))
    rows = []
    for _, rec in manifest.iterrows():
        folder = os.path.join(ROOT, rec["folder"])
        try:
            p = subprocess.run([EXE, folder], capture_output=True, text=True,
                               timeout=120, encoding="utf-8", errors="replace")
            out = p.stdout
        except subprocess.TimeoutExpired:
            out = ""
        info = parse_summary(out)
        seg_dur = {ACT_ALIAS.get(k, "OTHER"): v for k, v in info["segments"].items()}
        dominant = max(seg_dur, key=seg_dur.get) if seg_dur else "NO_OUTPUT"
        stair_time = seg_dur.get("STAIRS_UP", 0.0) + seg_dur.get("STAIRS_DOWN", 0.0) \
                   + seg_dur.get("STAIRS_LANDING", 0.0)
        stair_time = stair_time if stair_time else 0.0
        pol_up = seg_dur.get("STAIRS_UP", 0.0)
        pol_down = seg_dur.get("STAIRS_DOWN", 0.0)
        rows.append({
            "folder": rec["folder"],
            "trial": rec["trial"],
            "true_activity": rec["true_activity"],
            "subject": rec["subject"],
            "duration_s": rec["duration_s"],
            "parsed_steps": info.get("steps", 0),
            "ref_steps": ref_step_count(folder),
            "loop_closure_m": info.get("loop_closure_m", np.nan),
            "dominant_activity": dominant,
            "t_walk": seg_dur.get("WALKING", 0.0),
            "t_run": seg_dur.get("RUNNING", 0.0),
            "t_stairs_up": pol_up,
            "t_stairs_down": pol_down,
            "t_stairs_total": stair_time,
            "t_stationary": seg_dur.get("STATIONARY", 0.0),
            "t_other": seg_dur.get("OTHER", 0.0),
        })
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(ROOT, "eval_results.csv"), index=False)
    print(f"Evaluated {len(df)} recordings -> eval_results.csv")

    # ===== TỔNG HỢP =====
    print("\n" + "=" * 78)
    print("ĐỘ CHÍNH XÁC PHÂN LOẠI HOẠT ĐỘNG (hoạt động trội so với nhãn thật)")
    print("=" * 78)
    acc = (df["dominant_activity"] == df["true_activity"]).mean()
    print(f"Chính xác tổng thể: {acc*100:.1f}%  ({(df['dominant_activity']==df['true_activity']).sum()}/{len(df)})")
    print()
    for label in ["STAIRS_DESCENT", "STAIRS_ASCENT", "WALKING_FLAT", "RUNNING", "STATIONARY"]:
        sub = df[df["true_activity"] == label]
        if len(sub) == 0:
            continue
        acc_l = (sub["dominant_activity"] == label).mean()
        print(f"  {label:16s}: {acc_l*100:5.1f}% đúng  ({(sub['dominant_activity']==label).sum():2d}/{len(sub):2d})  "
              f"| dự đoán trội: {sub['dominant_activity'].value_counts().to_dict()}")

    stair = df[df["true_activity"].isin(["STAIRS_ASCENT", "STAIRS_DESCENT"])]
    if len(stair):
        print("\nPHÂN CỰC LÊN/XUỐNG trên các lượt CẦU THANG (tỷ lệ thời gian):")
        for label in ["STAIRS_ASCENT", "STAIRS_DESCENT"]:
            sub = stair[stair["true_activity"] == label]
            tot = sub["t_stairs_total"].sum()
            up = sub["t_stairs_up"].sum()
            dn = sub["t_stairs_down"].sum()
            print(f"  {label:16s}: thời gian thang={tot:.0f}s | phân lên={up/tot*100 if tot else 0:.0f}% "
                  f"| phân xuống={dn/tot*100 if tot else 0:.0f}%")

    print("\nSỐ BƯỚC (C++ vs peak-detection độc lập, theo loại trial):")
    for label in ["WALKING_FLAT", "RUNNING", "STAIRS_ASCENT", "STAIRS_DESCENT"]:
        sub = df[df["true_activity"] == label].copy()
        sub = sub[sub["ref_steps"] > 0]
        if len(sub) == 0:
            continue
        err = (sub["parsed_steps"] - sub["ref_steps"]) / sub["ref_steps"] * 100
        print(f"  {label:16s}: median err={err.median():+.1f}%  mean={err.mean():+.1f}%  "
              f"(n={len(sub)}, C++ median={sub['parsed_steps'].median():.0f}, ref median={sub['ref_steps'].median():.0f})")

if __name__ == "__main__":
    main()
