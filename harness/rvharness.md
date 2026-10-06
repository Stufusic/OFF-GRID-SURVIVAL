# CODE REVIEW — THƯ MỤC `harness/` (KHUNG THỰC NGHIỆM)

**Ngày review:** 06/09/2026 (sau Sprint S2) | **Phạm vi:** `harness/src/` + `harness/include/survival_harness/`

---

## 1. TỔNG QUAN

5 module: `csv_loader` (nạp dataset thật), `data_player` (replay), `failure_injector` (tiêm lỗi M0–M5), `metrics_collector` (NEES/NIS/CE50/CE95), `benchmark_runner` (factorial + Monte Carlo), `synthetic_generator` (dữ liệu tổng hợp).

## 2. ĐIỂM MẠNH

- **Metrics toán học đúng** (sau fix H3): NEES = eᵀP⁻¹e chuẩn, NIS scalar đúng, CE50/CE95 nearest-rank, containment 95% — được test thống kê phủ (Monte Carlo 10k → containment 93–97%, NEES ≈ 2.0).
- **Determinism nghiêm túc**: seed truyền tường minh (`environment_seed + run_idx*100`), reseed trong setConfig/resetSeed.
- **Synthetic generator tự nhất quán**: quy ước heading/gyro đúng (`heading_dot = −gyro_z`), mag projection khớp, công thức baro chuẩn, timestamp không tích lũy drift.
- **M4 giờ đo được thật** (M12): tiêm nhiễu IMU ±1.5 m/s² + gyro ±0.15 rad/s trong `injectSensorFrame`.

## 3. LỖI ĐÃ FIX (S2)

- **M11** `csv_loader.cpp`: BOM strip (3 vị trí); bounds-check `gt_easting/northing` (hết OOB crash); `tryParseDouble` + đếm lỗi (>5% → nullopt); `gtheadingdeg` × `kDegToRad`.
- **M12** `failure_injector.cpp`: M4 = IMU noise inflation (trước đây chỉ `injectStepEvent` không ai gọi → M4 ≡ M0).
- **M13** `benchmark_runner.cpp`: `high_resolution_clock` → `steady_clock` (MinGW = system_clock, không monotonic).

## 4. VẤN ĐỀ CÒN TỒN ĐỌNG

1. **[MEDIUM] Tiêu chí M2 không khớp tài liệu** — `benchmark_runner.hpp:55` ghi "CE95 tăng ≤ 20% so với M0" nhưng `benchmark_runner.cpp:154` chỉ check `mean_ce95_m <= 120.0` tuyệt đối; **không có baseline M0 nào được chạy** để so relative. Header :56 cũng lệch code (50 µs/2 ms vs 150 µs/10 ms).
2. **[LOW-MEDIUM] Không có warm-up exclusion** — các frame đầu gồm lazy-init làm latency statistics lệch.
3. **[LOW-MEDIUM] `loadSensorLoggerFolder`** (`csv_loader.cpp:466-501`): mag áp vào không có bound staleness (stream mag dừng sớm → heading stale nuôi tới hết recording); không check/sort monotonic thời gian; có thể chọn sample tương lai khi stream bắt đầu sau accel.
4. **[LOW] CSV headerless** mất row đầu (`read3DSeries/readScalarSeries` coi dòng đầu là header vô điều kiện); `auto_detect_header = false` parse header thành data zeros.
5. **[LOW] `data_player`**: step event có timestamp sau frame cuối không bao giờ fire (loop 22-41).
6. **[LOW] Độ dài kịch bản M2**: 2 hPa/h × ~1 phút mô phỏng = 0.25 m — chưa đủ để đo drift (khuyến nghị dataset 30+ phút, xem baseline_v232.md §3).
7. **[LOW] Degenerate run** (`benchmark_runner.cpp:128-131`): push CE50/CE95/NEES = 0 khi run không có sample → kéo trung bình Monte Carlo xuống ảo.

## 5. KHUYẾN NGHỊ

- Triển khai đúng tiêu chí M2 relative-to-M0 (chạy M0 baseline trước, so % tăng) + sửa doc header khớp code.
- Thêm cột `gnss_speed` vào CSV mapping để khai thác B4 với dataset thật.
- Thêm warm-up exclusion (bỏ N frame đầu khỏi latency stats).

**Điểm đánh giá chung: 7.5/10** — sau S2, độ tin cậy đo lường đã đảm bảo (đơn vị đúng, kịch bản M4 thật); còn nợ tiêu chí M2 relative và vài edge-case parser.
