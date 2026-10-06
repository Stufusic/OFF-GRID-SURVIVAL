# CODE REVIEW — THƯ MỤC `tests/` (BỘ KIỂM THỬ)

**Ngày review:** 06/09/2026 (sau Sprint S3) | **Phạm vi:** `tests/*.cpp` (40 file) + `tests/CMakeLists.txt`

---

## 1. TỔNG QUAN

Một executable `unit_tests` chứa 40 file test, GoogleTest 1.14, `gtest_discover_tests` → chạy riêng từng test case (cho phép parallel `-j`). **Hiện trạng: 182/182 PASS × 3 lần chạy liên tiếp** (hành trình: 118 → 137 (audit v2.2) → 160 (Critical/High) → 170 (S1) → 179 (S2) → 182 (S3)).

## 2. ĐIỂM MẠNH

- **Phủ tốt các module lõi**: packet serializer (roundtrip bit-exact), ESKF (PSD floor, gate, hội tụ Bắc), PF (propagation/resample/OOB culling/resize), DEM (bilinear, corruption v2), Jean Meeus, step/stride/PDR, FSM/vehicle/vertical transport, governor/auditor, C API (lifecycle + concurrency stress 1W/2R), replay determinism, fuzz corner cases.
- **S1–S3 đã vá các khoảng trống test quan trọng nhất**: `LocalBlackbox` (trước đây 0 test — nơi từng sinh Bug 10) nay có 6 test gồm torn-write recovery; Invariant 2 (no Null-Island) có regression; DEM corruption (5 test); NEES đơn vị (3 test thống kê); C-API đa reader stress; QNH bias hội tụ; declination; M4 tiêm IMU thật.
- **Test thống kê đúng chất**: `NeesUnits.ConsistentFilterContainmentNear95Pct` dùng Monte Carlo 10k với Cholesky — kiểm chứng containment 93–97% và NEES ≈ 2.0, không chỉ check công thức.

## 3. ĐIỂM YẾU CẤU TRÚC

1. **[MEDIUM] 40 file chung 1 binary** — crash/abort ở một test (fuzz, C API) làm mất kết quả toàn bộ phần sau trong process đó. Khuyến nghị: tách tối thiểu 2 binary (core-pure vs c_api/harness) hoặc để `gtest_discover` (đang chạy per-case, riêng crash giữa các case vẫn côlidate được).
2. **[LOW] Test ngưỡng hiệu năng theo wall-clock** — `test_spatial_hash_benchmark.cpp:64` `EXPECT_LT(elapsed_ms, 5.0)` (tolerance 25× lỏng hơn release, flaky trên CI tải cao).

## 4. KHOẢNG TRỐNG CÒN LẠI

1. **Invariant 3 (platform isolation)** — chỉ check thủ công bằng grep, chưa có test tự động (khuyến nghị: test biên dịch hoặc script CI grep `windows.h|android/*` trong core/src + core/include trừ c_api).
2. **Invariant 5 (monotonic uncertainty)** — chưa có sweep CE95/A95 non-decreasing qua GPS dropout (S09 kiểm tra handoff, không monotonicity).
3. **Invariant 6 (degraded fail-safe)** — `grep kDegraded|kLost tests/` vẫn không có assert chế độ hạ cấp end-to-end (M4/M5 test stale flag nhưng chưa assert chuyển modeLost khi NaN covariance).
4. **Desktop GUI** — 0 test (logic to_screen/ellipse đáng tách khỏi Win32 để test).
5. **Vacuous test còn sót** — `test_audit_v22_deep_dive.cpp:182` assert panic_circling ban đầu = false (pass cả khi bỏ fix Bug 6); phần Bug 6 thật chỉ nằm ở `test_scenario_harness.cpp` S14 (đi vòng quanh, dùng auditor trực tiếp, không qua C API).
6. **Nhân bản waveform** — Bug 3 flat-walk được phủ 3 lần gần giống hệt (phase1, audit, scenario S01) — hợp lệ như contrast nhưng nên gom helper chung.

## 5. ĐÁNH GIÁ CHẤT LƯỢNG TEST MỚI (S1–S3)

- Test có ý nghĩa phân biệt (discriminating): `GrowPreservesDistributionSpread` (bắt được C1: 8000 hạt trùng), `DeviceDropWithImpactSpinIsNotHumanFall` (M8a), `ImmobilityEscalationFiresBeforeCountdown` (M8b — dùng config pre_alert 300 s để countdown không che khuất), `BomFileTimestampStillMapped` (M11), `StraddleBoundaryMergesToOneCluster` (B1), `TwoReadersOneWriterStress` (H5).
- Bài học đã ghi: test baro phải vượt độ phân giải float của áp suất (~0.1 m @ 1013 hPa); test descent không thể phân biệt khi KHÔNG có bất kỳ tín hiệu phân biệt nào (đã bỏ test tiền đề vô hiệu, thay bằng pipeline drift test).

**Điểm đánh giá chung: 8/10** — coverage lõi tốt và các test mới đều discriminating; còn nợ invariants 3/5/6, desktop, và cấu trúc single-binary.
