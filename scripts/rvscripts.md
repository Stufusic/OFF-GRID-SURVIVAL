# CODE REVIEW — THƯ MỤC `scripts/` (PHÂN TÍCH DỮ LIỆU THẬT — PYTHON)

**Ngày review:** 06/09/2026 | **Phạm vi:** 32 script `.py` + 2 file CSV artifact

---

## 1. TỔNG QUAN

Bộ script phân tích một lần (one-off) phục vụ nghiên cứu dữ liệu cầu thang thật (Dataset 1/2 — ngày 2026-09-01): trích đặc trưng bước chân, tái dựng quỹ đạo 3D tầng/landing, kiểm chứng hướng quay, phân tích gyro bias/compass, replay tự hành. Nền: numpy/pandas/scipy/matplotlib.

## 2. ĐIỂM MẠNH

- **Giá trị nghiên cứu thật**: `reconstruct_true_3d_stairs.py`, `solve_stairs_physics_truth.py`, `find_true_turnaround_d1/d2.py` là nguồn ground-truth tầng/bậc đã hiệu chuẩn cho stair detector + `stair_geometry_estimator` (riser 0.17 m trong evaluation_results.txt đến từ đây).
- Quaternion → rotation matrix viết đúng (`replay_autonomous_stairs.py:12-26`, Hamilton, đối xứng).
- Đặt tên rõ mục đích (inspect_/diagnose_/find_/replay_/solve_), mỗi script một câu hỏi phân tích — tuân theo quy trình nghiên cứu Plan.md Phase 1.

## 3. VẤN ĐỀ

1. **[MEDIUM] Hardcoded path cá nhân** — hầu hết script khai `C:\Users\KieuKhang\Downloads\...` (ví dụ `replay_autonomous_stairs.py:6-9`) → không tái lập được trên máy khác/CI. Fix: nhận `sys.argv` hoặc biến môi trường `SURVIVAL_DATASET_DIR`, mặc định về thư mục `datasets/` theo cấu trúc Plan.md.
2. **[LOW] CSV artifact nằm chung source** — `dataset1_steps_detailed.csv` + `flight_analysis_results.csv` là OUTPUT nên nằm ở `docs/EXPERIMENT_REPORTS/` hoặc `export/` (hiện thư mục `export/` đang trống, sinh ra đúng mục đích này).
3. **[LOW] Trùng lặp logic** — quaternion conversion, peak detection, plotting setup bị copy ở ~10 script; nên gom `scripts/common/` (quat_to_rot, load_dataset, plot_style) để sửa một chỗ.
4. **[LOW] Không seed/timestamp output** — một số script ghi đè plot/png cùng tên giữa các lần chạy dataset khác nhau.
5. **[INFO] Phụ thuộc scipy/matplotlib** — hợp lý cho phân tích; nếu cần chạy trên CI chỉ các script replay thuần numpy thì tách requirements.

## 4. KHUYẾN NGHỊ

1. Tham số hóa path (1 giờ làm) — điều kiện để kết quả phân tích tái lập được.
2. Chuyển CSV artifact sang `export/`; tạo `scripts/README.md` bảng phân rã script → câu hỏi nghiên cứu → dataset → output.
3. Sau khi hiệu chuẩn xong, giữ lại 4–5 script giá trị dài hạn (replay_autonomous, reconstruct_true_3d, solve_physics_truth, inspect_gyro_bias) — phần còn lại lưu nhánh archive.

**Điểm đánh giá chung: 6.5/10** — giá trị nghiên cứu cao, kỹ thuật code chấp nhận được cho one-off; nợ tính tái lập (path) và trùng lặp helper.
