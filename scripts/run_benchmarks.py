"""Benchmark thống nhất cho toàn dự án: chạy replay_evaluation.exe trên
(1) 2 dataset tự thu (Sensor Logger, có barometer/mag) và
(2) MotionSense (360 recordings có nhãn hoạt động do người gán).

Đọc bản tóm tắt CSV (--csv) của tool — không regex bảng text.
Kết quả: docs/<label>_metrics_motionsense.csv, docs/<label>_metrics_own.csv,
docs/<label>_summary.txt (in ra stdout đồng thời).

Dùng làm mốc so sánh trước/sau mỗi phase cải tiến:
    python run_benchmarks.py --label baseline
    python run_benchmarks.py --label phase1 [--subset 40]
"""
import argparse
import os
import subprocess
import numpy as np
import pandas as pd
from scipy import signal

ROOT = r"C:\New PJ\OFF-GRID SURVIVAL"
EXE = os.path.join(ROOT, "build", "tools", "replay_evaluation.exe")
MS_ROOT = os.path.join(ROOT, "testdata", "motionsense")
DOCS = os.path.join(ROOT, "docs")

OWN_DATASETS = [
    ("D1_10-57-01_stairs_loop", r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1"),
    ("D2_11-24-39_jog_loop", r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"),
]

# Tham chiếu vật lý đã xác nhận của dự án (replay_both_datasets_detailed.py):
# D1: 145 bước, leo 72 / xuống 73 (đổi vế ~ nửa thời lượng), net 2D ~2.5m
# D2: 100 bước, leo 50 / xuống 50, net 2D ~2.1m
OWN_TRUTH = {
    "D1_10-57-01_stairs_loop": {"steps": 145, "turn_frac": 0.50, "net_2d_m": 2.54, "end_z_m": 0.0},
    "D2_11-24-39_jog_loop": {"steps": 100, "turn_frac": 0.46, "net_2d_m": 2.06, "end_z_m": 0.0},
}

MS_LABELS = {"dws": "STAIRS_DESCENT", "ups": "STAIRS_ASCENT",
             "wlk": "WALKING_FLAT", "jog": "RUNNING",
             "sit": "STATIONARY", "std": "STATIONARY"}


def ref_step_count(folder: str, fs: float) -> int:
    df = pd.read_csv(os.path.join(folder, "TotalAcceleration.csv"))
    cols = {"x": "x", "y": "y", "z": "z"}
    norm = np.sqrt(df["x"]**2 + df["y"]**2 + df["z"]**2).values
    if len(norm) < fs:
        return 0
    b, a = signal.butter(4, [0.7/(fs/2), 4.0/(fs/2)], btype="band")
    f = signal.filtfilt(b, a, norm)
    peaks, _ = signal.find_peaks(f, distance=int(0.35*fs), prominence=2.0)
    return len(peaks)


def run_tool(folder: str, csv_path: str) -> None:
    p = subprocess.run([EXE, folder, "--csv", csv_path],
                       capture_output=True, text=True, timeout=300,
                       encoding="utf-8", errors="replace")
    if p.returncode != 0:
        raise RuntimeError(f"replay_evaluation failed on {folder}: {p.stderr[:200]}")


def dominant_and_polarity(df_rows: pd.DataFrame, duration_s: float, turn_frac: float):
    """Trả về (hoạt động trội, {t_stairs_up, t_stairs_down, up_in_first_half})
    từ các dòng CSV của 1 dataset."""
    if len(df_rows) == 0:
        return "NO_OUTPUT", {}
    seg = df_rows.groupby("activity", as_index=False).agg(
        dur=("activity_duration_s", "sum"))
    dom = seg.loc[seg["dur"].idxmax(), "activity"]
    t_up = float(seg.loc[seg["activity"] == "STAIRS_ASCENT", "dur"].sum())
    t_dn = float(seg.loc[seg["activity"] == "STAIRS_DESCENT", "dur"].sum())
    turn_t = duration_s * turn_frac
    up_first = float(df_rows.loc[(df_rows["activity"] == "STAIRS_ASCENT") &
                                 (df_rows["activity_start"] < turn_t),
                                 "activity_duration_s"].sum())
    return dom, {"t_stairs_up": t_up, "t_stairs_down": t_dn, "up_first_half": up_first}


def eval_own(label: str):
    rows, summary = [], []
    for name, folder in OWN_DATASETS:
        if not os.path.isdir(folder):
            print(f"[SKIP] {name}: không tìm thấy {folder}")
            continue
        tmp_csv = os.path.join(DOCS, f"_own_{name}.csv")
        run_tool(folder, tmp_csv)
        df = pd.read_csv(tmp_csv)
        # activity_start: reconstruct từ duration tích lũy (CSV không có start)
        # -> dùng thứ tự dòng: start_k = cumsum(dur trước đó)
        df = df.reset_index(drop=True)
        df["activity_start"] = df["activity_duration_s"].shift(1).fillna(0.0).cumsum()
        truth = OWN_TRUTH[name]
        duration_s = float(df["duration_s"].iloc[0])
        dom, pol = dominant_and_polarity(df, duration_s, truth["turn_frac"])
        r = df.iloc[0]
        step_err = (int(r["steps"]) - truth["steps"]) / truth["steps"] * 100
        rows.append({
            "dataset": name, "steps": int(r["steps"]), "ref_steps": truth["steps"],
            "step_err_pct": step_err, "duration_s": duration_s,
            "loop_closure_m": float(r["loop_closure_m"]), "ref_net_2d_m": truth["net_2d_m"],
            "cum_vertical_m": float(r["cumulative_vertical_m"]), "ref_end_z_m": truth["end_z_m"],
            "stair_steps": int(r["stair_step_count"]), "riser_m": float(r["riser_m"]),
            "landings": int(r["landing_count"]),
            "dominant_activity": dom,
            "t_stairs_up": pol["t_stairs_up"], "t_stairs_down": pol["t_stairs_down"],
            "up_in_first_half_s": pol["up_first_half"],
        })
        os.remove(tmp_csv)
    own = pd.DataFrame(rows)
    own.to_csv(os.path.join(DOCS, f"{label}_metrics_own.csv"), index=False)
    return own


def eval_motionsense(label: str, subset: int | None):
    manifest = pd.read_csv(os.path.join(MS_ROOT, "manifest.csv"))
    if subset and subset < len(manifest):
        # Lấy đều mỗi loại hoạt động
        per = max(1, subset // manifest["true_activity"].nunique())
        manifest = (manifest.groupby("true_activity", group_keys=False)
                    .head(per).reset_index(drop=True))
    rows = []
    for _, rec in manifest.iterrows():
        folder = os.path.join(MS_ROOT, rec["folder"])
        tmp_csv = os.path.join(DOCS, "_ms_tmp.csv")
        run_tool(folder, tmp_csv)
        df = pd.read_csv(tmp_csv)
        if len(df) == 0:
            rows.append({"folder": rec["folder"], "true_activity": rec["true_activity"],
                         "dominant_activity": "NO_OUTPUT"})
            continue
        seg = df.groupby("activity", as_index=False).agg(
            dur=("activity_duration_s", "sum"), steps=("activity_steps", "sum"))
        dom = seg.loc[seg["dur"].idxmax(), "activity"]
        r = df.iloc[0]
        fs = 50.0
        rows.append({
            "folder": rec["folder"], "true_activity": rec["true_activity"],
            "subject": rec["subject"], "duration_s": float(r["duration_s"]),
            "steps": int(r["steps"]),
            "ref_steps": ref_step_count(folder, fs),
            "loop_closure_m": float(r["loop_closure_m"]),
            "cum_vertical_m": float(r["cumulative_vertical_m"]),
            "t_stairs_up": float(seg.loc[seg["activity"] == "STAIRS_ASCENT", "dur"].sum()),
            "t_stairs_down": float(seg.loc[seg["activity"] == "STAIRS_DESCENT", "dur"].sum()),
            "t_walk": float(seg.loc[seg["activity"] == "WALKING_FLAT", "dur"].sum()),
            "t_run": float(seg.loc[seg["activity"] == "RUNNING", "dur"].sum()),
            "t_stationary": float(seg.loc[seg["activity"] == "STATIONARY", "dur"].sum()),
            "dominant_activity": dom,
        })
    ms = pd.DataFrame(rows)
    ms.to_csv(os.path.join(DOCS, f"{label}_metrics_motionsense.csv"), index=False)
    return ms


def summarize(label: str, own: pd.DataFrame, ms: pd.DataFrame) -> str:
    lines = [f"===== BENCHMARK SUMMARY [{label}] ====="]

    if len(own):
        lines.append("\n-- 2 dataset tự thu (ground truth tham chiếu dự án) --")
        for _, r in own.iterrows():
            lines.append(
                f"{r['dataset']}: bước {r['steps']}/{r['ref_steps']} ({r['step_err_pct']:+.1f}%) | "
                f"loop {r['loop_closure_m']:.2f}m (ref ~{r['ref_net_2d_m']}m) | "
                f"Z_end {r['cum_vertical_m']:+.2f}m (ref ~{r['ref_end_z_m']:.1f}m) | "
                f"landing {r['landings']} | "
                f"t_up {r['t_stairs_up']:.0f}s t_dn {r['t_stairs_down']:.0f}s | "
                f"up pha-đầu {r['up_in_first_half_s']:.0f}s")

    if len(ms):
        lines.append("\n-- MotionSense (nhãn hoạt động thật) --")
        ms["correct"] = ms["dominant_activity"] == ms["true_activity"]
        acc = ms["correct"].mean() * 100
        lines.append(f"Chính xác trội: {acc:.1f}% ({ms['correct'].sum()}/{len(ms)})")
        for lab in ["STAIRS_ASCENT", "STAIRS_DESCENT", "WALKING_FLAT", "RUNNING", "STATIONARY"]:
            sub = ms[ms["true_activity"] == lab]
            if len(sub) == 0:
                continue
            lines.append(f"  {lab:15s}: {(sub['correct'].mean()*100):5.1f}% đúng "
                         f"({sub['correct'].sum()}/{len(sub)}) "
                         f"| trội: {sub['dominant_activity'].value_counts().to_dict()}")
        mov = ms[ms["ref_steps"] > 0].copy()
        if len(mov):
            mov["err_pct"] = (mov["steps"] - mov["ref_steps"]) / mov["ref_steps"] * 100
            lines.append("Số bước (median err theo loại): " + " | ".join(
                f"{lab}: {mov[mov['true_activity']==lab]['err_pct'].median():+.1f}%"
                for lab in ["WALKING_FLAT", "RUNNING", "STAIRS_ASCENT", "STAIRS_DESCENT"]
                if (mov["true_activity"] == lab).any()))
    text = "\n".join(lines)
    with open(os.path.join(DOCS, f"{label}_summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="baseline")
    ap.add_argument("--subset", type=int, default=None,
                    help="Chỉ chạy N recordings MotionSense (cân bằng mỗi loại)")
    ap.add_argument("--skip-own", action="store_true")
    ap.add_argument("--skip-ms", action="store_true")
    args = ap.parse_args()

    os.makedirs(DOCS, exist_ok=True)
    own = pd.DataFrame() if args.skip_own else eval_own(args.label)
    ms = pd.DataFrame() if args.skip_ms else eval_motionsense(args.label, args.subset)
    print(summarize(args.label, own, ms))


if __name__ == "__main__":
    main()
