# KẾ HOẠCH FIX MEDIUM + CẢI TIẾN ĐỘ CHÍNH XÁC — v2.3.2

**Căn cứ:** Báo cáo kiểm thử 06/09/2026 + benchmark probe thực tế (CE50 23.8 m / CE95 30.5 m Full Fusion; NEES 0.24; M0–M4 identical).
**Phạm vi:** 15 bug MEDIUM (M1–M15) + 3 residual (R1–R3) + 5 cải tiến độ chính xác (B1–B5).
**Tổng effort:** ~9–10 ngày dev, chia 3 sprint. Mỗi mục = 1 commit + regression test.

---

# GÓI A — FIX BUG MEDIUM

## A.1 Nhóm Pipeline & Estimator (M1, M2, M3, M4, M5, R1) — ~2 ngày

### M1 [MEDIUM] `baro_v_z` spike 10× — lệch tần số lọc/gia tốc
**Vị trí:** `multi_modal_fusion_pipeline.cpp:88-103`
**Nguyên nhân:** LPF áp kế chỉ cập nhật ở frame baro (10 Hz) nhưng `prev_frame_baro_alt_m_` bị refresh **mỗi frame 100 Hz** → trên frame baro `baro_v_z = Δf/0.01 ≈ 10×` tốc độ thật; ở 9/10 frame còn lại = 0. Là nguyên nhân v_z dao động dữ dội ở đầu ra.

**Fix:** chỉ refresh `prev` khi có mẫu baro mới, và chia cho đúng khoảng thời gian baro:
```cpp
// AFTER
bool baro_frame_now = false;
if (frame.has_baro && frame.pressure_hpa > 300.0F && frame.timestamp_ns != last_baro_ts_ns_) {
    filtered_baro_alt_m_ = 0.95 * filtered_baro_alt_m_ + 0.05 * raw_alt;
    baro_frame_now = true;
}
double baro_v_z = 0.0;
if (baro_frame_now && has_initial_baro_ && last_baro_ts_ns_ > 0) {
    const double dt_baro = static_cast<double>(frame.timestamp_ns - last_baro_ts_ns_) * 1e-9;
    if (dt_baro > 0.005) {
        baro_v_z = (filtered_baro_alt_m_ - prev_baro_for_rate_m_) / dt_baro;  // đúng tốc độ thật
    }
}
if (baro_frame_now) {
    prev_baro_for_rate_m_ = filtered_baro_alt_m_;
    last_baro_ts_ns_ = frame.timestamp_ns;
}
```
**Test:** feed luồng 100 Hz với baro 10 Hz đi lên 0.5 m/s → `vertical_speed_m_s` ≈ 0.5 ± 0.15 ổn định (trước fix dao động 0↔5). Xóa members cũ `prev_frame_baro_alt_m_` hoặc đổi nghĩa.

### M2 [MEDIUM] Tốc độ hardcode stride 0.75 m
**Vị trí:** `multi_modal_fusion_pipeline.cpp:145` (tham số thứ 4 của `velocity_estimator_.update`)
**Fix:** cache sải bước thích ứng cuối cùng:
```cpp
// members: double last_adaptive_stride_m_{0.75};
// trong nhánh step detected (sau khi có stride_res):
last_adaptive_stride_m_ = stride_res.mean_length_m;
// tại chỗ gọi update:
static_cast<double>(last_adaptive_stride_m_),   // thay 0.75
```
**Test:** chạy bộ stride mô phỏng 1.5 m, cadence 2.2 Hz → speed ≈ 3.3 m/s (trước fix 1.65 — sai 2×).

### M3 [MEDIUM] `kCableTransport` không có motion policy
**Vị trí:** `multi_modal_fusion_pipeline.cpp:155-199` (thiếu nhánh) và :285-286 (nhịp emit)
**Fix:** thêm nhánh xử lý y hệt vehicle (tích phân PDR + `propagateContinuousMotion` + emit 5 Hz):
```cpp
} else if (act == MotionActivity::kCableTransport && !vehicle_state.is_stopped) {
    current_pdr_pos_.easting_m += vel_out.velocity_enu_m_s.x * dt;
    current_pdr_pos_.northing_m += vel_out.velocity_enu_m_s.y * dt;
    current_pdr_pos_.altitude_m += vel_out.velocity_enu_m_s.z * dt;
    pf_engine_.propagateContinuousMotion(
        vel_out.velocity_enu_m_s.x * dt, vel_out.velocity_enu_m_s.y * dt,
        vel_out.velocity_enu_m_s.z * dt, vel_out.horizontal_speed_m_s, dt, act);
}
```
Cập nhật cả điều kiện emit 5 Hz (thêm `kCableTransport` vào danh sách 200 ms tại :288). **Test:** mô phỏng cáp treo 5 m/s → hạt dịch chuyển khớp PDR (sai lệch < 30 m sau 60 s — như test Bug 1 cũ).

### M4 [MEDIUM] Watchdog staleness nội bộ dead code
**Vị trí:** `pipeline.cpp:320` (ghi đè `timestamp_ns` mỗi frame) + `integrity_monitor.cpp:134`
**Nguyên nhân:** `auditState(state, frame.timestamp_ns)` so thời điểm hiện tại với chính nó → không bao giờ stale. Gốc rễ: watchdog không thể phát hiện "hết frame" nếu chỉ chạy khi CÓ frame — cơ chế chính đã chuyển lên C-API (H5 fix). Fix nội bộ để còn giá trị: canonical state chỉ ghi trong nhánh `should_emit` (nó vốn là "trạng thái phát ra cuối"), nhờ đó `state.timestamp_ns` là mốc emit cuối → khi frame mới đến lệch > 1.5 s so với emit cuối (emit fallback 1 Hz nên lệch thực tế ≤ ~1.1 s — không false positive), `kFaultEstimatorStale` sống lại.
**Fix:**
```cpp
// pipeline.cpp: di chuyển 2 dòng này vào TRONG khối if (should_emit):
latest_canonical_state_.timestamp_ns = frame.timestamp_ns;
latest_canonical_state_.position_enu = current_pdr_pos_;
// ... (cả khối gán canonical, xem M5)
// và audit giữ nguyên như hiện tại — giờ nó so frame hiện tại vs emit cuối
```
**Test:** feed frame t = 0..5 s bình thường (không stale) rồi nhảy thẳng t = 10 s → status có `kFaultEstimatorStale`.

### M5 [MEDIUM] `latest_canonical_state_` chỉ đổ một nửa
**Vị trí:** `pipeline.cpp:320-343`
**Fix:** trong khối should_emit, gán đủ các trường đang mặc định:
```cpp
latest_canonical_state_.nav_mode          = uncertainty_state.nav_mode;      // kLost/kDegraded đúng thực tế
latest_canonical_state_.position_source   = uncertainty_state.position_source;
latest_canonical_state_.ce50_m            = uncertainty_state.clusters[0].ce50;   // xem tên trường thật
latest_canonical_state_.ce95_m            = uncertainty_state.clusters[0].ce95_semi_major_m;
latest_canonical_state_.ellipse_heading_rad = uncertainty_state.clusters[0].ellipse_heading_rad;
latest_canonical_state_.position_cov_2d   = /* từ B1 (cluster cov thật) hoặc current_pdr_cov_ */;
latest_canonical_state_.gyro_bias         = eskf_.getGyroBias();
latest_canonical_state_.motion_activity   = act;
```
**Test:** ép nav_mode = kLost (cắt GPS + NIS cao) → `getLatestCanonicalState().nav_mode == kLost` (trước fix luôn kGpsAided).

### R1 [residual Bug 3] Đi-xuống-không-baro vẫn cộng độ cao; gate 0.35 lọt
**Vị trí:** `stair_motion_detector.cpp:240-242, 345` (guard/gate), `:246-296` (softmax thiên ascent), `pipeline.cpp:223-235`
**Fix 3 lớp:**
1. Guard cap 0.35 → **0.30** (dưới ngưỡng gate 0.35, `p_stair < 0.35` mới chặn được).
2. Cân bằng softmax hướng: nâng trọng số kênh delta-z (hiện 0.15) lên **0.40**, hạ rise-time (0.30) còn **0.15**, freefall (0.25) còn **0.20**; kênh delta-z giữ clamp [0.10, 0.90] → tổng hướng chuyển thành trung tính, descent với dz âm được nhận đúng.
3. Pipeline chỉ cộng `step_height` khi FSM hoạt động xác nhận stairs **VÀ** (có baro hoặc stair confidence > 0.8):
```cpp
const bool stair_confirmed = (act == MotionActivity::kStairsAscent || act == MotionActivity::kStairsDescent)
                             && (frame.has_baro || stair_state.confidence > 0.8F);
```
**Test:** 20 bước ĐI XUỐNG mô phỏng không baro → tổng delta altitude ≤ +0.3 m (trước fix +3.4 m); 20 bước đi phẳng không baro → không vào kStairsAscent.

## A.2 Nhóm Classifier & Detector (M6, M7) — 0.5 ngày

### M6 [MEDIUM] Baro rate chết gần mực nước biển
**Vị trí:** `motion_activity_classifier.cpp:68` (`prev_baro_alt_ > 1.0F`)
**Fix:** thay gate theo độ cao bằng flag "đã có mẫu baro":
```cpp
// members: bool has_baro_datum_{false};
if (!has_baro_datum_) { has_baro_datum_ = true; prev_baro_alt_ = baro_alt; prev_baro_timestamp_ns_ = frame.timestamp_ns; }
else if (prev_baro_timestamp_ns_ > 0 && frame.timestamp_ns > prev_baro_timestamp_ns_) { /* tính rate */ }
```
**Test:** baro quanh 0 m và −60 m (áp cao) → elevator/escalator vẫn nhận được.

### M7 [MEDIUM] Riser trung bình chia sai số hạng
**Vị trí:** `stair_geometry_estimator.cpp:22-36`
**Fix:** tách 2 biến đếm: `consecutive_flight_steps_` (tổng bước) và `valid_baro_delta_count_` (bước có delta hợp lệ); `avg_riser = flight_baro_accum_m_ / max(valid_baro_delta_count_, 1)`.
**Test:** 10 bước flight, 3 delta hợp lệ 0.17 m → avg = 0.17 (trước fix 0.051 → bị clamp 0.12).

## A.3 Nhóm Safety (M8, M9) — 1 ngày

### M8a [MEDIUM] Tumble window bao gồm mẫu va chạm → rơi máy bị nhận nhầm ngã người
**Vị trí:** `safety_auditor.cpp:39-65`
**Fix:** push mẫu hiện tại vào history SAU khi quét, hoặc quét bỏ phần tử vừa push:
```cpp
// đẩy trước nhưng quét từ phần tử CŨ NHẤT tới phần tử TRƯỚC mẫu vừa push:
const size_t n_prior = (gyro_count_ > 1) ? gyro_count_ - 1 : 0;   // loại mẫu hiện tại
for (size_t i = 0; i < n_prior; ++i) { ... }                      // giữ nguyên điều kiện cửa sổ 300ms
```
**Test:** rơi máy thẳng đứng (impact 6g, gyro < 3 rad/s trước impact) → không PreAlert; ngã người có tumble 4 rad/s trong 300 ms trước impact → PreAlert.

### M8b [MEDIUM] Escalation bất động 180 s dead code
**Vị trí:** `safety_auditor.cpp:78-89` (45 s countdown luôn thắng 180 s; predicate bất động sai)
**Fix:** tách 2 cơ chế độc lập trong nhánh kPreAlert:
```cpp
// 1) Countdown 45s giữ nguyên
// 2) Bất động: cửa sổ trượt tính từ LẦN CỬ ĐỘNG CUỐI (không phải từ lúc ngã)
if (timestamp_ns - last_motion_ns_ >= config_.immobility_threshold_ns) {
    state_ = SafetyState::kDistressLocked;
}
```
Lưu ý điều kiện `timestamp_ns >= last_motion_ns_` (clock có thể không monotonic trong replay — guard như H5 style).
**Test:** ngã → cử động tại t=60 s → nằm yên → DistressLocked tại ~t=240 s (trước fix không bao giờ nổ); ngã → cử động liên tục → không escalate.

### M9 [MEDIUM] SMS cứu hộ tọa độ (0,0) — Null Island
**Vị trí:** `survival_c_api.cpp:368-395` (encode 0/0 khi không có fix) + `sos_egress_manager.cpp:33-46`
**Fix:** dùng bit MSB của byte `status_flags` trong packet 160-bit làm cờ **NO_FIX** (hiện byte này chưa dùng bit nào khác — verify khi làm):
```cpp
// C-API: xác định has_fix = (frame origin có datum) || lastKnownGoodFix.is_valid
const uint8_t status = static_cast<uint8_t>(current_state.nav_mode) | (has_fix ? 0x00 : 0x80);
// sos_egress_manager::buildCompactSmsPayload:
const bool no_fix = (status_flags_byte & 0x80) != 0;   // cần decode trả status_flags
if (no_fix) {
    std::snprintf(buffer, sizeof(buffer), "SOS | NO-FIX | NAV:%s | BAT:%.0f%% | CE95:%.0fm", mode_str, battery_soc, c0.ce95_semi_major_m);
    return buffer;
}
```
Cần cho `DecodedRescuePacket`/decode trả về `status_flags` (thêm trường). **Test:** engine chưa có fix → SMS không chứa "LAT:" và không chứa "maps.google.com"; có fix → bit 0x80 = 0, SMS có tọa độ.

## A.4 Nhóm Governor (M10) — 0.5 ngày

### M10 [MEDIUM] Không hysteresis tại 20/5/2% → mode flapping
**Vị trí:** `resource_governor.cpp:20-60`
**Fix:** biên进出 lệch nhau 3% (enter khi tụt xuống X, thoát khi lên X+3):
```cpp
// Vào mode thấp hơn: ngưỡng như cũ (20 / 5 / 2)
// Thoát lên mode cao hơn: phải vượt X + 3
PowerMode desired = ...;  // theo bảng hiện tại
if (desired < current_mode_ && last_soc_ < exit_threshold_of(current_mode_)) { current_mode_ = desired; }
// hoặc đơn giản: tính desired từ bảng hai chiều:
//   Balanced   <-> Conservative: xuống khi <=20, lên khi >=23
//   Conservative <-> Critical:   xuống khi <=5,  lên khi >=8
//   Critical   <-> DisplayOnly:  xuống khi <=2,  lên khi >=5
```
Giữ nguyên nhánh `is_emergency` (override tức thì, không hysteresis). `state.target_particle_count`... chỉ xuất khi mode thực sự đổi hoặc luôn xuất từ `current_mode_` (luôn xuất — stateless hơn cho consumer).
**Test:** quét SoC dao động sin biên độ 2% quanh 20 → số lần đổi mode ≤ 2 (trước fix dao động mỗi update); quét 100→0→100 trơn → đúng 3 lần xuống + 3 lần lên.

## A.5 Nhóm Harness & Platform (M11, M12, M13, M14, M15) — 1.5 ngày

### M11 [MEDIUM] CSV loader 4 lỗi
**Vị trí:** `csv_loader.cpp:382-395` (OOB), `:17-26` (BOM), `:53-63` (parse), `:392-395` (deg→rad)
**Fix:**
```cpp
// 1. OOB: gộp điều kiện bounds cho gt_easting/northing
if (mapping.gt_easting_idx >= 0 && mapping.gt_easting_idx < static_cast<int>(tokens.size())
    && mapping.gt_northing_idx >= 0 && mapping.gt_northing_idx < static_cast<int>(tokens.size())) { ... }

// 2. BOM: trong normalizeHeader()
if (col.size() >= 3 && static_cast<unsigned char>(col[0]) == 0xEF
    && static_cast<unsigned char>(col[1]) == 0xBB && static_cast<unsigned char>(col[2]) == 0xBF) {
    col.remove_prefix(3);
}

// 3. parse fail: đổi parseDouble thành trả std::optional<double>; caller đếm parse_errors_
//    và (tùy cấu hình) bỏ row hoặc gán NaN; khi > 5% rows lỗi → loadFromFile trả nullopt.

// 4. Deg->rad cho gtheadingdeg:
double hdg = parseDouble(tokens[...]);
if (mapping.gt_heading_is_deg) { hdg *= kDegToRad; }   // mapColumns set flag khi header là "gtheadingdeg"
dataset.ground_truth_headings_rad.push_back(hdg);
```
**Test:** CSV Excel (có BOM) → timestamp mapping đúng; row ngắn → không crash, đếm lỗi; header `gtheadingdeg` giá trị 90 → `ground_truth_headings_rad ≈ π/2`.

### M12 [MEDIUM] Kịch bản M4 no-op trong benchmark
**Vị trí:** `benchmark_runner.cpp:78` (chỉ gọi injectSensorFrame) + `failure_injector.cpp:53-64` (injectStepEvent không ai gọi)
**Fix (đổi cơ chế M4 sang IMU noise inflation — đo được mà không cần bước中间 step events):** trong `FailureInjector::injectSensorFrame`, nhánh kM4:
```cpp
// M4 = PDR noise: nhiễu trắng gia tốc + jitter yaw đo được
if (scenario == ScenarioType::kM4_PdrNoiseInjection && inWindow(ts)) {
    frame.accel_m_s2[2] += gauss(rng_) * 1.5F;             // nhiễu dọc 1.5 m/s²
    frame.accel_m_s2[0] += gauss(rng_) * 0.8F;
    frame.gyro_rad_s[2] += gauss(rng_) * 0.15F;            // jitter yaw ~8.6°/s — sai heading PDR
}
```
(Giữ `injectStepEvent` cho test đơn vị; document ý nghĩa M4 mới trong header.) **Test:** M4 với nhiễu ON vs OFF → CE95(M4) > CE95(M0) có ý nghĩa thống kê; report không còn identical từng chữ số.

### M13 [MEDIUM] `high_resolution_clock` không monotonic trên MinGW
**Vị trí:** `benchmark_runner.cpp:81,83`
**Fix:** `std::chrono::steady_clock` thay `high_resolution_clock` (2 dòng).

### M14 [MEDIUM] Win32 ANSI API fail path tiếng Việt
**Vị trí:** `platform_memory_map.cpp:88,107` (`CreateFileA`, `CreateFileMappingA`), `platform_file_system.cpp:28` (`CreateFileA`)
**Fix:** helper chuyển UTF-8 → UTF-16 rồi gọi W-API:
```cpp
static std::wstring utf8ToWide(const std::string& s) noexcept {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w(static_cast<size_t>(n > 0 ? n - 1 : 0), L'\0');
    if (n > 0) MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
    return w;
}
// CreateFileW(utf8ToWide(path).c_str(), ...) / CreateFileMappingW(...)
```
**Test:** tạo file tên có dấu (`bản_đềm_test.sdem`) → load DEM thành công qua mmap + stream.

### M15 [MEDIUM] `initializePlatformHooks` không gọi trong production → blackbox không fsync
**Vị trí:** `platform_file_system.cpp:63-71` (chỉ test gọi) + `survival_engine_create`
**Fix:** trong `survival_engine_create` (survival_c_api.cpp), gọi một lần idempotent:
```cpp
survival_handle_t survival_engine_create(void) {
    platform::initializePlatformHooks();   // idempotent — thêm static guard nếu chưa có
    auto* ctx = new (std::nothrow) EngineContext();
    ...
}
```
**Test:** sau create → `LocalBlackbox` nhận fsync thật (kiểm qua PAL hook hoặc spy).

---

# GÓI B — CẢI TIẾN ĐỘ CHÍNH XÁC (B1–B5) — ~4 ngày

### B1 [ROI CAO NHẤT] Ellipse cụm theo hiệp phương sai THẬT + merge ô kề
**Vị trí:** `particle_filter_engine.cpp:422-423` (cluster_var cố định `(cell*0.5)²` = 2500 m² → CE95 ~122 m cố định) + `extractUncertaintyState`
**Hiện trạng:** mọi cluster báo CE95 ≈ 122.4 m bất kể phân tán thật; NEES sụp 0.24; CE95 của Full Fusion phản ánh ô lưới chứ không phải chất lượng filter (chính là thứ làm số 30.5 m trong probe mất nghĩa).
**Fix:**
1. Với mỗi cluster (ô có hạt): tính **weighted mean + weighted covariance 2D** thật của các hạt trong ô (var_e, var_n, cov_en) → `computeUncertaintyEllipse(var_e, var_n, cov_en)` → bán trục/heading/area theo đúng phân tán.
2. **Merge ô kề** (8-neighborhood) thành 1 giả thuyết bằng flood-fill trên các cell có hạt — hết hiện tượng 1 giả thuyết straddle biên ô bị tách làm 2; hai giả thuyết khác nhau cách < 1 ô không bị gộp nhầm (flood-fill chỉ gộp ô kề nhau trực tiếp, mỗi cụm 1 id).
3. Với cụm < 5 hạt: giữ floor hiệp phương sai (1 m²) tránh ellipse ảo 0.
**Kỳ vọng định lượng (test acceptance):** cloud std 10 m → CE95 báo ≈ 24.5 m (±20%); cloud std 2 m → CE95 ≈ 5 m; NEES synthetic về khoảng [1.0, 3.0]; số cluster giảm khi 1 đám mây straddle biên.
**Test:** `extractUncertaintyState` trên cloud Gaussian biết trước std → ellipse khớp; straddle test: cloud tâm nằm trên biên 2 ô → active_clusters_count == 1.

### B2 [LỚN NHẤT về thuật toán] Bias áp kế QNH (thời tiết) làm state ước lượng
**Vị trí:** `pipeline.cpp` (filtered_baro_alt_m_ so DEM trực tiếp tại `updateWeights` call, σ=3.5 m)
**Hiện trạng:** baro tính từ QNH 1013.25 hPa cố định; thời tiết thật lệch 10–50 m (3–14σ) → likelihood cao độ kéo đám mây tới SAI cao độ vĩnh viễn; chưa có state bias nào.
**Fix:** ước lượng bias chậm khi filter tự tin:
```cpp
// members: double baro_bias_m_{0.0};  bool bias_ready_{false};
// trong khối should_emit (1-5 Hz, đủ chậm):
const auto dem_alt = dem_.getElevationBilinear(center.easting_m, center.northing_m);
if (dem_alt && pf_confident) {   // N_eff > 0.8N && active_clusters == 1
    const double innov = filtered_baro_alt_m_ - (*dem_alt + baro_bias_m_);
    if (std::abs(innov) < 30.0) {                       // gate outlier (chặn snap sai)
        baro_bias_m_ += 0.002 * innov * dt_emit;        // EMA chậm ~ vài phút
        bias_ready_ = true;
    }
}
// khi cấp cho updateWeights:
pf_engine_.updateWeights(dem_, filtered_baro_alt_m_ - baro_bias_m_, ...);
```
`pf_confident` lấy từ uncertainty_state hiện có (score/N_eff). Clamp bias ±120 m. **Không bật** khi multi-cluster (không biết cụm nào thật).
**Kỳ vọng:** mô phỏng QNH lệch +30 m (bão) → bias hội tụ 30 m trong < 10 phút, CE95 không phình > 1.5× (trước fix: lệch dần 30 m và kéo cụm xuống thấp).
**Test:** unit test áp bias ±30 m; kiểm tra bias KHÔNG cập nhật khi 2 cluster.

### B3 Từ sai (declination) thành cấu hình
**Vị trí:** `eskf_3d.cpp:227` (`innovation_yaw = atan2(-m_nav.x, m_nav.y)` giả định declination = 0)
**Fix:** thêm `double mag_declination_rad{0.0}` vào `EskfConfig`; innovation − declination; expose qua PipelineConfig → C-API (hàm `survival_engine_set_declination`). Chỉnh lệch thực địa Việt Nam ~ −1°..0° (nhỏ) nhưng Tây Bắc/nước khác tới 10–20°.
**Test:** quaternion heading thật 10° + declination 5° → heading đầu ra 10° (không 5°) khi config đúng.

### B4 Thêm trường GNSS speed vào SensorFrame + fusion
**Vị trí:** `data_contracts.hpp:13-35` (SensorFrame không có speed) → `pipeline.cpp:149` truyền cứng −1.0F
**Fix:** thêm `float gnss_speed_m_s{-1.0F}` vào SensorFrame (field cuối, cập nhật static offset nếu cần); pipeline truyền thẳng vào `velocity_estimator_.update` (tham số đã có sẵn gnss_speed) → phục vụ: cycling/cable gate có tốc độ thật (gỡ R2 đúng gốc), ZUPT/negative-update khi GNSS báo 0.
**Test:** frame với gnss_speed 6 m/s + cadence 1.5 Hz → activity kCycling đạt được qua FSM (trước fix bất khả thi).

### B5 Chuẩn hóa lại toàn bộ đo lường sau khi gói A+B1 xong
- Tái chạy factorial probe (M0–M4, 20 MC × 120 bước) → bảng CE50/CE95/NEES thật sự khác nhau giữa kịch bản (M12).
- NEES mục tiêu [1.0, 3.0] (B1 + H3).
- Chạy `tools/replay_evaluation.cpp` trên Dataset 1/2 có ground truth → xuất CE95 thật địa (bổ sung ghép GPS log làm truth nếu thiếu).
- Ghi kết quả vào `docs/EXPERIMENT_REPORTS/` làm baseline v2.3.2.

---

# LỘ TRÌNH THỰC HIỆN (3 SPRINT)

| Sprint | Nội dung | Effort | Điều kiện nghiệm thu |
|---|---|---|---|
| **S1 (A.1–A.4)** | M1–M10 + R1 core fixes | ~4 ngày | ctest 100%; CE95 stair-descent-no-baro ≤ 0.3 m; baro_v_z ổn định; SMS no-fix |
| **S2 (A.5 + B1)** | Harness/platform (M11–M15) + cluster cov thật (B1) | ~2.5 ngày | NEES ∈ [1.0, 3.0]; CE95 phản ánh phân tán thật; M0≠M4 |
| **S3 (B2–B5)** | QNH bias + declination + GNSS speed + tái chuẩn hóa đo lường | ~3 ngày | QNH ±30 m không làm phình CE95; bảng benchmark v2.3.2 công bố được |

**Definition of Done chung:** build sạch (tận dụng bật `-Werror` ở sprint này), ctest 100% (~160 → ~185 test), mỗi mục 1 commit kèm test, cập nhật `docs/audit_report` checklist.

**Rủi ro chính:**
- B1 đổi hình dạng ellipse → desktop GUI/BLE hiển thị CE95 nhỏ hơn rõ rệt (đúng thực tế, cần thông báo nếu có consumer nhạy ngưỡng).
- B2 bias áp kế có nguy cơ "hấp thụ" cả sai số thật nếu gate pf_confident lỏng — gate N_eff + single-cluster + |innov| < 30 m là bắt buộc.
- M4 (watchdog) phụ thuộc việc chuyển canonical update vào nhánh emit — phải chạy ctest toàn bộ vì nhiều test đọc canonical sau frame đơn lẻ (như test H2 — sẽ cần điều chỉnh nếu bị ảnh hưởng).
