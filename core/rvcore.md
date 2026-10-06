# CODE REVIEW — THƯ MỤC `core/` (LÕI THUẬT TOÁN)

**Ngày review:** 06/09/2026 (sau Sprint S1+S2+S3) | **Reviewer:** Kiểm toán toàn diện + 3 sprint fix
**Phạm vi:** `core/src/` + `core/include/survival_core/` — 10 module con: c_api, comms, environment, estimators, integrity, kinematics, math, platform (PAL interface), safety, storage

---

## 1. TỔNG QUAN

Lõi C++20 thuần của hệ điều hướng xác suất: ESKF 3D, Particle Filter TRN, PDR, DEM/Jean Meeus, phân loại hoạt động, an toàn (fall/SOS), blackbox, gói cứu hộ 160-bit, C API lock-free.

**Trạng thái:** 182/182 unit tests PASS · build sạch không warning (trừ desktop GUI) · 3 lần chạy liên tiếp ổn định.

## 2. ĐIỂM MẠNH (đã kiểm chứng độc lập)

- **Toán học lõi chính xác**: hệ số Butterworth bandpass (0.5–3 Hz @ 100 Hz), Jacobian PDR `[L,ψ]→[E,N]`, systematic resampling (không off-by-one), Joseph-form ESKF, Hamilton quaternion, Jean Meeus (azimuth đúng quy ước Bắc — tránh bẫy +180°), CRC32 IEEE 802.3, static_spatial_hash (an toàn tọa độ âm).
- **Zero-allocation hot path** được tuân thủ nghiêm trong kinematics/PF (mảng tĩnh, pre-allocated).
- **Invariant 3 (Core Isolation)**: core không include OS header; `platform_factory.hpp` chỉ là declaration thuần (được thêm PRIVATE include cho c_api ở S2/M15, symbol resolve ở tầng link).
- **C API**: triple-buffer 1W/NR (mutex reader-only, writer lock-free 100%), atomic hóa trạng thái chéo luồng, watchdog freshness 3 tầng (5s/1.5s/0.5s).

## 3. LỖI ĐÃ FIX (theo 3 sprint — chi tiết trong docs/fix_plan_*.md)

| Nhóm | Đã fix |
|---|---|
| Audit v2.2 (12 lỗi) | Bug 1–12 (8 fix đúng gốc, 4 fix hoàn thiện nốt ở S1) |
| Critical/High (7) | C1 sụp phân phối `setTargetParticleCount`; H1 trục Pitch sai (classifier + ESKF euler); H2 WGS-84 rad→deg; H4 DEM validate trước cấp phát + NaN-nodata; H5 C-API data race; H6 blackbox flush/format v2 |
| S1 (M1–M10+R1) | baro_v_z zero-order hold; stride thích ứng; cáp treo policy; watchdog stale; canonical đầy đủ; flat-guard cap + softmax cân bằng + pitch-conflict; baro datum; riser divisor; tumble/immobility; SMS no-fix; governor hysteresis |
| S2 | M11–M15 harness/platform; B1 ellipse hiệp phương sai thật + merge ô kề (NEES 0.24 → 12.6 đo được) |
| S3 | B2 bias áp kế QNH (EMA τ≈100s, gate tự tin); B3 declination config + C API; B4 GNSS speed field |

## 4. VẤN ĐỀ CÒN TỒN ĐỌNG (theo thứ tự ưu tiên)

1. **[MEDIUM] Hiệp phương sai PDR nền quá nhỏ** — NEES Pure PDR = 320 (baseline_v232.md): `stride_variance`/`heading_variance` nền không phủ sai số hệ thống khi không có DEM. Khuyến nghị: nâng variance nền khi `active_clusters_count == 0`.
2. **[MEDIUM] Elevator < 1.6 m/s không vào nổi FSM** (`motion_transition_fsm.cpp` threshold 0.75; elevator score 0.70+0.25·(vz−1)/3): thang máy dân dụng 0.5–1.5 m/s bị ZUPT-lock. Khuyến nghị: hạ entry threshold cho elevator khi baro-rate ổn định.
3. **[MEDIUM] `kCableTransport` chưa đạt end-to-end** — policy đã có (`multi_modal_fusion_pipeline.cpp` nhánh kCableTransport) nhưng gate 2.0 m/s chỉ mở được khi feed thật có `gnss_speed_m_s` (B4 đã nối plumbing; dataset CSV chưa có cột này).
4. **[LOW] `Eskf3D::updateZeroVelocity` là stub no-op** (`eskf_3d.cpp:299-302`); `updateZeroAngularRate` không được gọi; `zupt_noise_std` không dùng; `imu_health_` vĩnh viễn 1.0 (Invariant 7 nhận hằng số); `consecutive_mag_rejects_` tăng nhưng không đọc (chưa có escalation).
5. **[LOW] GUI mobile nằm trong core** — `core/CMakeLists.txt:30-31` biên dịch `fps_governor.cpp`/`ribbon_renderer.cpp` từ `mobile/` vào survival_core (vi phạm phân tầng về tinh thần, xem mobile/rvmobile.md).
6. **[LOW] Timestamp underflow** khi clock regression: `frame.timestamp_ns − last_output_timestamp_ns_` trên uint64 (`pipeline.cpp` nhánh emit) → should_emit mỗi frame. Đã guard ở chỗ khác, chỗ này chưa.
7. **[LOW] `loadFromStream` từ chối stream không seekable** — hợp lý cho DEM file nhưng cần ghi rõ trong contract.

## 5. KHUYẾN NGHỊ

- Bật `-Werror` + quyết định `-fno-exceptions` (Rule.md §I.3 yêu cầu nhưng CMake chưa set; mọi fix hiện tại đã thiết kế pre-validation nên an toàn khi bật).
- Tách mobile renderer ra khỏi survival_core trước khi dựng khung Android.
- Hiệu chỉnh stride/heading variance bằng dataset thật (Garmin/RTK) để hạ NEES Pure PDR.

**Điểm đánh giá chung: 8.5/10** — lõi thuật toán đúng, kiến trúc sạch, đã qua 3 vòng kiểm toán + fix; còn nợ hiệu chuẩn variance bằng dữ liệu thật.
