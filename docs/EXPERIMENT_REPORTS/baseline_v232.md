# BASELINE ĐO LƯỜNG v2.3.2 (sau Sprint S1 + S2 + S3)

**Ngày:** 06/09/2026 | **Cấu hình:** 20 Monte Carlo × 120 bước chân, SyntheticGenerator, seed chuẩn
**Trạng thái mã nguồn:** sau S3 (B2 QNH bias + B3 declination + B4 GNSS speed + B1 ellipse thật)

## Kết quả Full Fusion (ESKF + PDR + DEM + Thiên văn)

| Kịch bản | CE50 [m] | CE95 [m] | NEES | M1 pass | M2 pass |
|---|---|---|---|---|---|
| M0 Nominal | 31.13 | 45.42 | 12.56 | ✅ | ✅ |
| M1 Magnetic Spike +45 µT | 31.14 | 45.43 | 12.57 | ✅ | ✅ |
| M2 Baro Drift 2 hPa/h | 31.13 | 45.42 | 12.56 | ✅ | ✅ |
| M3 Celestial Outage | 31.13 | 45.42 | 12.56 | ✅ | ✅ |
| M4 IMU Noise (±1.5 m/s²) | 31.33 | 45.59 | 12.74 | ✅ | ✅ |
| M5 Combined | 31.40 | 45.62 | 12.84 | ✅ | ✅ |

## So sánh chế độ (M0)

| Chế độ | CE50 [m] | CE95 [m] | NEES |
|---|---|---|---|
| Pure PDR | 17.04 | 32.18 | 319.9 |
| ESKF + PDR | 17.04 | 32.18 | 319.9 |
| Full Fusion | 31.13 | 45.42 | 12.56 |

## Hiệu năng

- Fast loop: ~67 µs (ngưỡng nghiệm thu 150 µs) ✅
- Particle step: ~0.78 ms (ngưỡng 10 ms) ✅

## Diễn giải & việc còn lại

1. **NEES Full Fusion = 12.6** (kỳ vọng lý tưởng 2.0): filter **hơi tự tin** — sai số trung tâm cụm so với ground truth gồm thành phần hệ thống chung (stride/heading bias của cả đám) không nằm trong spread của hạt. Đây là chỉ số cần theo dõi khi hiệu chỉnh bằng dataset thật; đứng ở 12.6 là đo được và ổn định (trước B1: 0.24 — không đo được gì).
2. **Pure PDR NEES = 320**: hiệp phương sai PDR đang nhỏ hơn sai số thật ~160× — cần nâng `stride_variance`/`heading_variance` nền khi không có DEM (hạng mục Medium còn lại).
3. **M1 ≈ M0 là ĐÚNG thiết kế**: NIS-gating chặn spike +45 µT (Invariant 4). M2 cần thời lượng mô phỏng dài hơn (2 hPa/h × 1 phút = 0.03 hPa ≈ 0.25 m) — khuyến nghị chạy M2 với dataset 30+ phút khi có.
4. **CE95 45 m ở 120 bước** tương đương dự đoán thực địa 30–80 m (kịch bản mất GPS, địa hình có đặc trưng) — khớp đánh giá trong báo cáo kiểm thử.

## Hành trình metric NEES (chứng minh chuỗi đo lường đã được sửa)

| Thời điểm | NEES Full Fusion | Nguyên nhân |
|---|---|---|
| Trước H3 | ≈ 0 (bão hòa) | Truyền bán trục (m) vào tham số phương sai (m²) |
| Sau H3, trước B1 | 0.24 | Ellipse cụm cố định theo ô lưới (122 m) |
| Sau B1 (hiện tại) | 12.56 | Hiệp phương sai thật — đo được |
