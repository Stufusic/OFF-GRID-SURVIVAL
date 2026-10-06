"""Chuyển MotionSense dataset (GitHub: mmalekzadeh/motion-sense, A_DeviceMotion_data)
sang format Sensor Logger folder mà CsvLoader::loadSensorLoggerFolder đọc được.

Đơn vị nguồn (CoreMotion qua SensingKit): gravity & userAcceleration = [g],
rotationRate = [rad/s]. Total accel (m/s^2) = (userAcc + gravity) * 9.80665.
Cột `time` = nanosecond, dt = 20ms (50Hz) — khớp cách loader parse (time * 1 ns).

Không có từ kế trong MotionSense -> bỏ Magnetometer (frame.has_mag=false, ESKF
sẽ trôi yaw — chấp nhận, các chỉ số bước/cầu thang không phụ thuộc heading).

Vì loader chỉ map cột `time` + x/y/z, mỗi sensor ghi header: time,x,y,z
"""
import os
import sys
import numpy as np
import pandas as pd

SRC = r"C:\New PJ\OFF-GRID SURVIVAL\testdata\A_DeviceMotion_data"
DST = r"C:\New PJ\OFF-GRID SURVIVAL\testdata\motionsense"
G = 9.80665
DT_NS = 20_000_000  # 50 Hz

TRUE_LABEL = {
    "dws": "STAIRS_DESCENT",
    "ups": "STAIRS_ASCENT",
    "wlk": "WALKING_FLAT",
    "jog": "RUNNING",
    "sit": "STATIONARY",
    "std": "STATIONARY",
}

def convert_file(src_csv: str, dst_dir: str) -> int:
    df = pd.read_csv(src_csv)
    if len(df) < 200:
        return 0
    # LƯU Ý dataset: gravity của CoreMotion chỉ cập nhật ~1Hz (bậc thang) trong khi
    # userAcceleration 50Hz — tổng accel có artifact bậc thang nhỏ gây bước ma ~1Hz
    # trên đoạn đứng yên. Đã thử LP gravity (tệ hơn tổng thể) — giữ nguyên bản gốc.
    ax = (df["userAcceleration.x"] + df["gravity.x"]).values * G
    ay = (df["userAcceleration.y"] + df["gravity.y"]).values * G
    az = (df["userAcceleration.z"] + df["gravity.z"]).values * G
    t = np.arange(len(df), dtype=np.int64) * DT_NS  # ns

    os.makedirs(dst_dir, exist_ok=True)
    pd.DataFrame({"time": t, "x": ax, "y": ay, "z": az}).to_csv(
        os.path.join(dst_dir, "TotalAcceleration.csv"), index=False, float_format="%.6f")
    pd.DataFrame({"time": t,
                  "x": df["rotationRate.x"].values,
                  "y": df["rotationRate.y"].values,
                  "z": df["rotationRate.z"].values}).to_csv(
        os.path.join(dst_dir, "Gyroscope.csv"), index=False, float_format="%.6f")
    return len(df)

def main():
    os.makedirs(DST, exist_ok=True)
    manifest = []
    n_conv = 0
    for trial in sorted(os.listdir(SRC)):
        trial_dir = os.path.join(SRC, trial)
        if not os.path.isdir(trial_dir) or "_" not in trial:
            continue
        prefix = trial.split("_")[0]
        for subj_file in sorted(os.listdir(trial_dir)):
            if not subj_file.endswith(".csv"):
                continue
            subject = subj_file.replace("sub_", "").replace(".csv", "")
            dst_dir = os.path.join(DST, f"{trial}_sub{subject}")
            n = convert_file(os.path.join(trial_dir, subj_file), dst_dir)
            if n == 0:
                os.rmdir(dst_dir)
                continue
            manifest.append({
                "folder": f"{trial}_sub{subject}",
                "trial": trial,
                "subject": int(subject),
                "true_activity": TRUE_LABEL[prefix],
                "duration_s": n / 50.0,
            })
            n_conv += 1
    pd.DataFrame(manifest).to_csv(os.path.join(DST, "manifest.csv"), index=False)
    print(f"Converted {n_conv} recordings -> {DST}")
    print(f"Duration range: {min(m['duration_s'] for m in manifest):.1f}s .. "
          f"{max(m['duration_s'] for m in manifest):.1f}s")

if __name__ == "__main__":
    main()
