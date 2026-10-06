import shutil
import os

src_dir = r"c:\New PJ\OFF-GRID SURVIVAL\docs"
art_dir = r"C:\Users\KieuKhang\.gemini\antigravity-ide\brain\e0461dc4-411b-4e9b-aaf0-f4e1be00a05a"

files = [
    "true_3d_stairs_2026-09-01_10-57-01_1.png",
    "true_3d_stairs_2026-09-01_11-24-39_1.png",
    "coaxial_stairwell_2026-09-01_10-57-01_1.png",
    "dataset1_true_turnaround.png",
    "dataset2_true_turnaround.png"
]

for f in files:
    s = os.path.join(src_dir, f)
    d = os.path.join(art_dir, f)
    if os.path.exists(s):
        shutil.copy2(s, d)
        print(f"Copied {f} to artifact directory")
