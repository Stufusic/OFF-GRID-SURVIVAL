# KẾ HOẠCH FIX BUG MỨC CRITICAL & HIGH — v2.3.1

**Căn cứ:** Báo cáo kiểm thử toàn diện (06/09/2026) — 1 CRITICAL + 6 HIGH.
**Phạm vi:** 7 lỗi (C1, H1–H6). Mỗi lỗi = 1 commit riêng + regression test. Tổng effort dự kiến: 4–5 ngày dev.
**Quy trình chung cho từng mục:** fix code → thêm/sửa test → build sạch (`cmake --build`) → `ctest` 100% → commit.

---

## THỨ TỰ THỰC HIỆN (theo độ rủi ro lan dvá test)

| Bước | Bug | Nội dung | Effort |
|---|---|---|---|
| 1 | H2 | Đơn vị WGS-84 (1 dòng) | 0.5h |
| 2 | H1 | Trục Pitch sai (ESKF + Classifier) | 1 ngày |
| 3 | C1 | `setTargetParticleCount` sụp phân phối | 0.5 ngày |
| 4 | H3 | NEES truyền mét vào tham số m² | 0.5 ngày |
| 5 | H4 | DEM validate trước khi cấp phát | 0.5 ngày |
| 6 | H6 | Blackbox flush nói dối + format | 1 ngày |
| 7 | H5 | C-API data race (mutex reader + atomic) | 1 ngày |

Lý do thứ tự: H2/H1 là sửa toán học nền — làm trước để các test chạy sau ổn định; H5 để cuối vì phạm vi rộng nhất (đa luồng).

---

## 1. H2 [HIGH] — Tọa độ WGS-84 lưu RADIANS vào trường DEGREES

**Vị trí:** `core/src/estimators/multi_modal_fusion_pipeline.cpp:322-324`
**Nguyên nhân:** `DemMap::enuToLatLon()` trả `{lat_rad, lon_rad}` (dem_map.cpp:408-417) bị gán thẳng vào `position_wgs84.latitude_deg/longitude_deg` → sai 57.29×.

**Fix (multi_modal_fusion_pipeline.cpp:322-324):**
```cpp
// BEFORE
latest_canonical_state_.position_wgs84.latitude_deg  = wgs_lat;
latest_canonical_state_.position_wgs84.longitude_deg = wgs_lon;

// AFTER — kRadToDeg có sẵn: types.hpp:20
const auto [wgs_lat_rad, wgs_lon_rad] = dem_.enuToLatLon(current_pdr_pos_.easting_m, current_pdr_pos_.northing_m);
latest_canonical_state_.position_wgs84.latitude_deg  = wgs_lat_rad * kRadToDeg;
latest_canonical_state_.position_wgs84.longitude_deg = wgs_lon_rad * kRadToDeg;
```
**KIỂM TRA KHÔNG ĐỘNG vào:** `particle_filter_engine.cpp:122-123` (truyền rad cho `JeanMeeusSolver` — đúng, giữ nguyên).

**Test mới** (`tests/test_multi_modal_fusion.cpp` hoặc file mới `test_canonical_state.cpp`):
```cpp
TEST(CanonicalStateWgs84, DegreesNotRadians) {
    // DEM origin tại Hà Nội (21.0285°, 105.8542°), đi bộ +500 m về hướng Bắc
    // assert: |lat_deg - (21.0285 + 500/6371000*kRadToDeg)| < 1e-3
    // assert: |lon_deg - 105.8542| < 1e-3
    // assert: (lat,lon) != (0,0)  — đóng luôn khoảng trống regression test Invariant 2
}
```
**Nghiệm thu:** test trên PASS; không test nào cũ fail.

---

## 2. H1 [HIGH] — "Pitch" đang đo nghiêng NGANG (trục +X) thay vì đổ người VỀ TRƯỚC

**Vị trí:** `core/src/kinematics/motion_activity_classifier.cpp:62-63` và `core/src/estimators/eskf_3d.cpp:360-373`
**Nguyên nhân:** Frame BODY chuẩn dự án là **+X=Phải, +Y=Trước, +Z=Lên** (`docs/frame_conventions.md` §II). Công thức `asin(2(w·y − z·x))` là độ cao của trục **+X (phải)** = roll/bank ngang, không phải pitch. Toàn bộ gate `pitch_rad > 0.08` (stair ascent/descent, flat suppression) đang khóa nhầm tín hiệu.

**Quy ước đích (định nghĩa lại, ghi chú vào `docs/frame_conventions.md`):**
- `pitch_rad`: **dương = trục trước chúc xuống (thân đổ về trước)** — đúng ngữ nghĩa các gate đang dùng.
- `roll_rad`: **dương = nghiêng sang phải**.

**Fix A — motion_activity_classifier.cpp:62-63:**
```cpp
// BEFORE
const double sinp = 2.0 * (attitude_q.w * attitude_q.y - attitude_q.z * attitude_q.x);
const double pitch_rad = std::asin(std::clamp(sinp, -1.0, 1.0));

// AFTER — độ cao trục trước: sin(elev) = R_nb(2,1) = 2(yz + wx); pitch = −elev
const double fwd_elev_sin = 2.0 * (attitude_q.y * attitude_q.z + attitude_q.w * attitude_q.x);
const double pitch_rad = -std::asin(std::clamp(fwd_elev_sin, -1.0, 1.0));
```

**Fix B — eskf_3d.cpp (khối Roll/Pitch trong extractAttitude):**
```cpp
// BEFORE
const double sinr_cosp = 2.0 * (w * x + y * z);
const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
est.euler.roll_rad = std::atan2(sinr_cosp, cosr_cosp);
const double sinp = 2.0 * (w * y - z * x);
...

// AFTER — frame Y-forward/Z-up
// Roll (bank quanh trục trước +Y): dương = nghiêng phải  => roll = atan2(−R(2,0), R(2,2))
est.euler.roll_rad = std::atan2(2.0 * (w * y - x * z), 1.0 - 2.0 * (x * x + y * y));
// Pitch (đổ về trước quanh trục phải +X): dương = chúc xuống => pitch = −asin(R(2,1))
const double fwd_elev_sin = 2.0 * (y * z + w * x);
if (std::abs(fwd_elev_sin) >= 1.0) {
    est.euler.pitch_rad = std::copysign(kHalfPi, -fwd_elev_sin);
} else {
    est.euler.pitch_rad = -std::asin(fwd_elev_sin);
}
```
*(Định lý kiểm chứng: nghiêng phải thuần góc φ → quaternion (cos φ/2, 0, sin φ/2, 0) → R(2,0) = −sinφ, R(2,2) = cosφ → roll = +φ ✓. Đổ trước thuần góc θ (xoay quanh trục phải) → R(2,1) = −sinθ → pitch = +θ ✓.)*

**Việc kèm bắt buộc:**
1. `grep -rn "euler\.\|pitch_rad\|roll_rad" core/ desktop/` — rà consumer: HUD desktop hiển thị euler (đổi nhãn/giá trị nếu cần), các gate trong classifier giữ nguyên dấu vì quy ước mới đúng ngữ nghĩa cũ.
2. Cập nhật `docs/frame_conventions.md` — bổ sung định nghĩa roll/pitch mới kèm hình vẽ ASCII.
3. **Cập nhật test cũ** đang assert giá trị euler theo công thức sai (`test_eskf_3d.cpp`, `test_quaternion_conventions.cpp`).

**Test mới/điều chỉnh:**
```cpp
TEST(QuaternionConventions, ForwardLeanIsPositivePitch) {
    // Quaternion xoay quanh trục phải (body X) làm trục trước chúc xuống 30°
    // assert: pitch_rad ≈ +0.5236, roll_rad ≈ 0, heading không đổi
}
TEST(QuaternionConventions, RightBankIsPositiveRoll) {
    // Quaternion bank phải 20° -> roll ≈ +0.349, pitch ≈ 0
}
TEST(ActivityClassifier, LateralCamberDoesNotTriggerStairs) {
    // Đi bộ phẳng có dao động ROLL (đường nghiêng ngang), pitch = 0
    // -> không boost stair, activity = kWalkingFlat (trước khi fix sẽ false-positive)
}
```
**Rủi ro:** test cũ gắn với công thức sai sẽ fail → phải sửa expectation theo vật lý đúng (đây là mục đích, không phải hồi quy).

---

## 3. C1 [CRITICAL] — `setTargetParticleCount` phá hủy phân phối hạt

**Vị trí:** `core/src/estimators/particle_filter_engine.cpp:246-260`
**Nguyên nhân:** Grow = `resize(n, particles_[0])` → nhân bản hạt [0] thành 8000 hạt giống hệt nhau (point-mass, N_eff=N nên resample/roughening không bao giờ chạy, CE95 ảo, Invariant 7 violation). Shrink = cắt N phần tử đầu mảng (thiên lệch). Được kích hoạt chắc chắn bởi `survival_engine_update_battery_soc` (survival_c_api.cpp:424, target=10000 khi pin khỏe).

**Fix — viết lại toàn bộ hàm (resample theo trọng số + jitter khi grow):**
```cpp
void ParticleFilterEngine::setTargetParticleCount(size_t count) noexcept {
    static constexpr size_t kMinParticles = 500;
    static constexpr size_t kMaxParticles = 20000;
    const size_t safe_count = std::clamp(count, kMinParticles, kMaxParticles);
    if (safe_count == particles_.size() || particles_.empty()) {
        return;
    }

    // (1) Chuẩn hóa trọng số cục bộ (caller có thể gọi giữa chu kỳ update)
    double w_total = 0.0;
    for (const auto& p : particles_) { w_total += p.weight; }
    if (!(w_total > 0.0) || !std::isfinite(w_total)) { w_total = static_cast<double>(particles_.size()); }

    // (2) Systematic resampling N -> safe_count theo trọng số hậu nghiệm (giữ đúng phân phối)
    resample_buffer_.resize(safe_count);
    std::uniform_real_distribution<double> u_dist(0.0, 1.0);
    const double r0 = u_dist(rng_) / static_cast<double>(safe_count);
    const double inv_w_total = 1.0 / w_total;
    double cum = particles_.front().weight * inv_w_total;
    size_t src = 0;
    for (size_t m = 0; m < safe_count; ++m) {
        const double target = r0 + static_cast<double>(m) / static_cast<double>(safe_count);
        while (cum < target && src + 1 < particles_.size()) {
            ++src;
            cum += particles_[src].weight * inv_w_total;
        }
        resample_buffer_[m] = particles_[src];
    }

    // (3) GROW: jitter Gauss nhẹ phá bản sao trùng (chống suy biến point-mass)
    if (safe_count > particles_.size()) {
        const double grow_ratio = static_cast<double>(safe_count) / static_cast<double>(particles_.size());
        const double sigma = std::min(3.0, std::max(0.5, 0.25 * std::sqrt(grow_ratio))); // [m]
        std::normal_distribution<double> jitter(0.0, 1.0);
        for (auto& p : resample_buffer_) {
            p.easting_m  += jitter(rng_) * sigma;
            p.northing_m += jitter(rng_) * sigma;
            p.altitude_m += jitter(rng_) * sigma * 0.5;
        }
    }

    particles_.swap(resample_buffer_);
    config_.num_particles = safe_count;

    const double uniform_w = 1.0 / static_cast<double>(safe_count);
    for (auto& p : particles_) {
        p.weight = uniform_w;
        p.log_weight = std::log(uniform_w);
    }
    n_eff_ = static_cast<double>(safe_count);
}
```
**Lưu ý thiết kế:**
- `resample_buffer_.resize()` cấp phát heap — chấp nhận được vì hàm chỉ chạy khi **sự kiện pin** (không nằm trong vòng 50 Hz). Ghi comment điều này vào code.
- Trọng số uniform sau resample là trạng thái chuẩn hậu-resample (như `resample()` hiện có).
- Nếu `w_total` vô hạn/0 (state hỏng giữa chừng) → fallback đều, không crash.

**Test mới** (`tests/test_particle_filter.cpp`):
```cpp
TEST(ParticleFilterResize, GrowPreservesDistributionSpread) {
    // init 2000 hạt std 20 m -> setTargetParticleCount(10000)
    // assert: std của đám mây >= 50% std ban đầu (không sụp point-mass)
    // assert: tỷ lệ hạt trùng tuyệt đối (E,N giống hệt) < 5%
    // assert: |mean mới − mean cũ| < 3 m (không dịch chuyển tâm)
}
TEST(ParticleFilterResize, ShrinkIsWeightedRepresentative) {
    // Tạo 2 cụm: cụm A (900 hạt, weight thấp), cụm B (100 hạt, weight cao ~0.9)
    // shrink 1000 -> 500: assert số hạt rơi vào vùng cụm B tỉ lệ thuận với weight của nó
    // (trước fix: cắt 500 hạt ĐẦU mảng — sai)
}
TEST(CApiGovernor, BatteryUpdateDoesNotCollapseCloud) {
    // create -> push frames -> update_battery_soc(90) (target 10000)
    // assert: ce95 area không co < 30% so với trước khi gọi
}
```

---

## 4. H3 [HIGH] — Benchmark truyền BÁN TRỤC (m) vào tham số PHƯƠNG SAI (m²) → NEES vô nghĩa

**Vị trí:** `harness/src/benchmark_runner.cpp:104-121` (nguồn gốc gọi: `metrics_collector.hpp:56-64`).
**Nguyên nhân:** `recordEstimate(..., c0.ce95_semi_major_m * 0.5, ...)` — 95% semi-axis ≈ 2.4477σ nên phương sai đúng là `(a/√5.9915)²`; P bị phình ~6-12× → NEES ≈ 0, containment_95 bão hòa ~100%. Nhánh PDR còn hardcode `10.0, 10.0`.

**Fix A — thêm hàm nghịch đảo elip→hiệp phương sai** vào `core/include/survival_core/math/statistics.hpp` (đối xứng với `computeUncertaintyEllipse` đã có):
```cpp
/**
 * @brief Chuyển elip 95% (a, b, heading) về hiệp phương sai 2D [[var_e, cov_en],[cov_en, var_n]]
 * heading_rad đo từ Bắc, quay theo chiều Đông (quy ước computeUncertaintyEllipse).
 */
[[nodiscard]] inline std::array<double, 4> ellipseToCov2D(
    double semi_major_m, double semi_minor_m, double heading_rad,
    double chi_square_factor = kChiSquare2D_95) noexcept {
    const double k = std::max(chi_square_factor, 1e-9);
    const double lam1 = (semi_major_m * semi_major_m) / k;  // λ dọc trục lớn
    const double lam2 = (semi_minor_m * semi_minor_m) / k;  // λ dọc trục nhỏ
    const double s = std::sin(heading_rad);
    const double c = std::cos(heading_rad);
    const double var_e = lam1 * s * s + lam2 * c * c;   // trục lớn: u = (sinθ, cosθ) trong (E,N)
    const double var_n = lam1 * c * c + lam2 * s * s;
    const double cov_en = (lam1 - lam2) * s * c;
    return {var_e, cov_en, cov_en, var_n};
}
```

**Fix B — benchmark_runner.cpp nhánh PF (dòng 118-120):**
```cpp
// BEFORE
c0.ce95_semi_major_m * 0.5, c0.ce95_semi_minor_m * 0.5, 0.0
// AFTER
const auto cov = survival::math::ellipseToCov2D(
    c0.ce95_semi_major_m, c0.ce95_semi_minor_m, c0.ellipse_heading_rad);
metrics.recordEstimate(frame.timestamp_ns, est_pos, gt, cov[0], cov[3], cov[1]);
```

**Fix C — nhánh PDR (dòng 104-109):** thêm accessor vào `multi_modal_fusion_pipeline.hpp` (sau getDeadReckoningPosition):
```cpp
[[nodiscard]] std::array<double, 4> getDeadReckoningCovariance() const noexcept {
    return current_pdr_cov_;   // [var_e, cov_en, cov_en, var_n] — pipeline.hpp:135
}
```
và benchmark dùng `pipeline.getDeadReckoningCovariance()` thay `10.0, 10.0, 0.0`.

**Fix D (tùy chọn, cùng commit):** truyền heading thật vào `recordEstimate` (2 tham số cuối đang default 0.0 → `mean_heading_error_deg` luôn 0 trong báo cáo) — `dataset.ground_truth_headings_rad` có sẵn; **đồng thời sửa N10: `csv_loader.cpp:392-395` phải × `kDegToRad` khi header là `gtheadingdeg`.**

**Test mới** (`tests/test_metrics_collector.cpp`):
```cpp
TEST(NeesUnits, EllipseToCovRoundtrip) {
    // cov -> computeUncertaintyEllipse -> ellipseToCov2D phải khôi phục đúng cov gốc ( sai số <1e-9)
}
TEST(NeesUnits, KnownErrorGivesKnownNees) {
    // lỗi (1,0), var_e=var_n=1, cov=0 -> NEES = 1.0 (±1e-9)
    // elip a=b=sqrt(5.9915), θ=0 -> cov đơn vị -> NEES của lỗi đơn vị = 1.0
}
TEST(NeesUnits, ConsistentFilterContainmentNear95Pct) {
    // Monte Carlo 10k mẫu: x ~ N(0, P) đúng, dùng P từ elip -> containment_95 ∈ [93%, 97%]
    // (trước fix: ~100% do P phình)
}
```
**Nghiệm thu thêm:** chạy lại 1 benchmark M0; `mean_nees_2d` trên ~50 runs phải nằm trong [0.5, 3.0] (hiện tại ~0).

---

## 5. H4 [HIGH] — DEM: cấp phát trước khi kiểm CRC trong hàm `noexcept` → `std::terminate` khi file hỏng

**Vị trí:** `core/src/environment/dem_map.cpp` — `loadFromStream:187-191` (nghiêm trọng), `loadFromMemoryMap:142-173` (thiếu validate), `loadFromMemory:78` (đủ w/h>0 nhưng thiếu khớp size payload).
**Nguyên nhân:** `width/height` đọc từ file không được kiểm tra trước `std::vector<float> grid(total_elements)` — file hỏng/gian lận → `bad_alloc`/`length_error` trong hàm `noexcept` → `std::terminate()`. `loadFromMemoryMap` còn thiếu validate `w/h>0, cell_size>0` → UB trong `std::clamp` (col0 clamp với hi = −1).

**Fix A — helper dùng chung (đặt đầu file, namespace ẩn):**
```cpp
namespace {
// Trần cứng 64 triệu ô (256 MB payload float) — chặn bad_alloc từ header giả mạo
constexpr uint64_t kMaxDemCells = 64ull * 1024ull * 1024ull;

// Trả về số byte payload khả dụng, 0 nếu header bất hợp lệ.
// available_payload_bytes = số byte thực tế còn lại sau header (stream/mmap/span).
bool validateSdemHeader(const SdemFileHeader& hdr, uint64_t available_payload_bytes) noexcept {
    if (hdr.magic != 0x5344454D || hdr.version != 1) return false;
    if (hdr.width_cells == 0 || hdr.height_cells == 0) return false;
    if (!(hdr.cell_size_m > 0.0F) || !std::isfinite(hdr.cell_size_m)) return false;
    const uint64_t cells = static_cast<uint64_t>(hdr.width_cells) * static_cast<uint64_t>(hdr.height_cells);
    if (cells > kMaxDemCells) return false;
    if (cells * sizeof(float) > available_payload_bytes) return false;  // file cụt
    return true;
}
}
```
**Fix B — loadFromStream:** sau khi đọc header, xác định kích thước còn lại **trước khi** cấp phát:
```cpp
is.seekg(0, std::ios::end);
const auto end_pos = is.tellg();
is.seekg(static_cast<std::streamoff>(sizeof(SdemFileHeader)), std::ios::beg);
if (end_pos < 0) { return false; }  // stream không seekable -> từ chối (an toàn)
const uint64_t available = static_cast<uint64_t>(end_pos) - sizeof(SdemFileHeader);
if (!validateSdemHeader(file_hdr, available)) { return false; }
// CHỈ SAU ĐÓ: std::vector<float> grid(total_elements); → đọc → CRC như cũ
```
**Fix C — loadFromMemoryMap & loadFromMemory:** gọi cùng helper với `available = span.size() - sizeof(SdemFileHeader)`; giữ nguyên CRC check sau. (CRC vẫn chạy sau vì cần đọc payload — nhưng giờ không còn đường dẫn cấp phát từ con số không kiểm soát.)

**Fix D (hardening rẻ tiền, gộp cùng commit):** `getElevationBilinear` — coi NaN là nodata: `if (std::isnan(z) || z <= nodata_threshold) return false;` (dem_map.cpp:350-353 hiện `NaN <= nodata` là false → NaN lọt qua).

**Test mới** (`tests/test_dem_map.cpp` bổ sung hoặc file mới `test_dem_corruption.cpp`):
```cpp
TEST(DemCorruption, GarbageDimensionsDoesNotCrash)      // header 0xFFFFFFFF mọi trường -> loadFromStream trả false
TEST(DemCorruption, TruncatedFileRejected)              // file cắt ngắn giữa payload -> false
TEST(DemCorruption, ZeroDimensionsWithValidCrcRejected) // w=h=0, CRC của payload rỗng hợp lệ -> false
TEST(DemMmap, ZeroDimMmapRejected)                      // loadFromMemoryMap zero-dim -> false (chặn clamp UB)
TEST(DemMap, NanCellTreatedAsNodata)                    // cell NaN -> getElevationBilinear = false
```
*Lưu ý: các hàm `noexcept` — giải pháp là PRE-VALIDATION (chặn đường throw), không phải try/catch.*

---

## 6. H6 [HIGH] — Blackbox: flush báo thành công khi ghi lỗi + format không có header/CRC

**Vị trí:** `core/src/storage/local_blackbox.cpp:31-59` + `local_blackbox.hpp`.
**Nguyên nhân:** (a) không kiểm tra `out.good()`/kết quả `hardwareSync`, luôn `unflushed_count_=0; return true` → mất record cứu hộ khi đĩa đầy; (b) `unflushed_count_++` vô hạn khi đĩa hỏng (dập open mỗi sample — còn dư địa Bug 10 cũ); (c) file raw 36B/record không magic/version/CRC → mất điện giữa chừng ghi lệch khung là toàn bộ phần sau không phục hồi được.

**Fix A — format v2 (local_blackbox.hpp):**
```cpp
#pragma pack(push, 1)
struct BlackboxFileHeader {
    uint32_t magic{0x31584242};          // "BBX1" (little-endian)
    uint16_t version{2};
    uint16_t record_size{40};            // sizeof(BlackboxWireRecord)
};
struct BlackboxWireRecord {
    BlackboxRecord record;               // 36 bytes
    uint32_t crc32{0};                   // CRC32 của 36 byte record
};
#pragma pack(pop)
static_assert(sizeof(BlackboxFileHeader) == 8, "...");
static_assert(sizeof(BlackboxWireRecord) == 40, "...");
```
Members mới: `size_t consecutive_flush_failures_{0}; bool file_header_written_{false};`

**Fix B — flushToDisk viết lại (local_blackbox.cpp):**
```cpp
bool LocalBlackbox::flushToDisk() noexcept {
    if (count_ == 0 || filepath_.empty()) { return false; }
    const size_t safe_unflushed = std::min(unflushed_count_, kCapacity);
    if (safe_unflushed == 0) { return true; }

    std::ofstream out(filepath_, std::ios::binary | std::ios::in | std::ios::out);
    if (!out.is_open()) { ++consecutive_flush_failures_; return false; }

    out.seekp(0, std::ios::end);
    if (out.tellp() <= 0 && !file_header_written_) {
        const BlackboxFileHeader hdr{};
        out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
        if (!out.good()) { ++consecutive_flush_failures_; return false; }
        file_header_written_ = true;
    }

    const size_t start_idx = (head_ + kCapacity - safe_unflushed) % kCapacity;
    for (size_t i = 0; i < safe_unflushed; ++i) {
        const size_t idx = (start_idx + i) % kCapacity;
        const BlackboxWireRecord wire{ring_buffer_[idx],
            math::Crc32::calculate(std::span<const uint8_t>(
                reinterpret_cast<const uint8_t*>(&ring_buffer_[idx]), sizeof(BlackboxRecord)))};
        out.write(reinterpret_cast<const char*>(&wire), sizeof(wire));
        if (!out.good()) { ++consecutive_flush_failures_; return false; }  // KHÔNG reset counter
    }
    out.flush();
    if (!out.good()) { ++consecutive_flush_failures_; return false; }

    const auto fs = platform_fs_ ? platform_fs_ : s_platform_fs_;
    if (fs) {
        if (fs->hardwareSync(filepath_) != platform::PlatformIoResult::kSuccess) {
            ++consecutive_flush_failures_; return false;                        // KHÔNG reset counter
        }
    }
    unflushed_count_ = 0;          // chỉ reset khi chắc chắn ghi + sync thành công
    consecutive_flush_failures_ = 0;
    return true;
}
```
**Fix C — pushRecord (clamp + backoff):**
```cpp
void LocalBlackbox::pushRecord(const BlackboxRecord& record) noexcept {
    ring_buffer_[head_] = record;
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity) { ++count_; }
    if (unflushed_count_ < kCapacity) { ++unflushed_count_; }   // KHÔNG tăng vô hạn nữa

    // Backoff luỹ thừa khi đĩa hỏng: 100 -> 200 -> 400 -> 800 -> 1600 mẫu (tránh dập open())
    const size_t shift = std::min(consecutive_flush_failures_, static_cast<size_t>(4));
    if (unflushed_count_ >= (kAutoFlushStepInterval << shift)) {
        flushToDisk();
    }
}
```
**Fix D — API phục hồi sau sự cố (thêm vào header + cpp):**
```cpp
// Đọc lại toàn bộ record hợp lệ từ file (bỏ qua bản ghi CRC sai / ghi lệch khung).
// Dùng cho hậu kiểm sau khi thiết bị mất điện / crash. Trả nullopt nếu file không hợp lệ.
[[nodiscard]] static std::optional<std::vector<BlackboxRecord>>
recoverRecords(const std::string& filepath) noexcept;
```
Logic: đọc 8-byte header (magic/version/record_size) → stride record_size, CRC từng record, record sai thì bỏ qua và tiếp tục (torn-write chỉ mất đúng record đó).

**Test mới** (`tests/test_local_blackbox.cpp` — file MỚI, module này chưa có test nào):
```cpp
TEST(BlackboxFlush, FailurePreservesUnflushedCount)     // filepath không ghi được -> flush=false, getUnflushedCount() giữ nguyên
TEST(BlackboxFlush, SuccessResetsCounterAndWritesCrc)   // flush OK -> unflushed=0; đọc lại file: 8B header + N*40B, CRC khớp
TEST(BlackboxFlush, BackoffReducesRetryRate)            // 10k push với đĩa hỏng -> số lần thử open ~ <= 10k/1600*k (đếm qua spy PAL)
TEST(BlackboxRecovery, TornWriteLosesOnlyOneRecord)     // hỏng 1 record giữa file -> recover trả N-1 record, các record sau vẫn đọc đúng
TEST(BlackboxRecovery, RejectsBadMagic)                 // file rác -> nullopt
TEST(BlackboxRing, WraparoundKeepsLatest5000)           // push 6000 -> count==5000, record 0..999 đã bị ghi đè đúng
```

---

## 7. H5 [HIGH] — C-API: data race (triple-buffer 1-reader vs 3 entry point; trường plain dùng chéo luồng)

**Vị trí:** `core/src/c_api/survival_c_api.cpp` — `LockFreeTripleBuffer` (:22-73, `read_idx_` non-atomic tại :31/:67), `last_sensor_frame_mono_ns` (:172-178 đọc tại :462), `simulated_offset_ns` (:489), `current_battery_soc` (:410 đọc tại :226/:326/:389). Header hứa "Thread-safe snapshot" (survival_c_api.h:103).
**Nguyên nhân:** (a) thiết kế 1 writer/1 reader nhưng 3 entry point cùng gọi `read()` → 2 thread đọc song song phá invariant ownership; (b) các biến chéo luồng là kiểu thường → UB theo C++20.

**Fix A — đọc có khóa, ghi giữ lock-free (sửa trong LockFreeTripleBuffer):**
```cpp
#include <mutex>
// ...
mutable std::mutex readers_mutex_;   // chỉ tuần tự hóa các READER với nhau; writer không bao giờ chạm

bool read(NavigationUncertaintyState& out_state) const {
    std::lock_guard<std::mutex> lock(readers_mutex_);   // <=3 reader, tần số thấp — chi phí không đáng kể
    // ... thân read() giữ nguyên (CAS acquire, copy) ...
}
```
Ghi chú thiết kế vào comment: writer vẫn lock-free 100%; mutex chỉ bảo vệ `read_idx_` giữa nhiều reader. Phương án từ chối: per-reader index (cần TLS xuyên biên C — phức tạp không cần thiết).

**Fix B — atomic hóa trạng thái chéo luồng (EngineContext):**
```cpp
// BEFORE
uint64_t last_sensor_frame_mono_ns{0};
uint64_t simulated_offset_ns{0};
double   current_battery_soc{100.0};

// AFTER
std::atomic<uint64_t> last_sensor_frame_mono_ns{0};
std::atomic<uint64_t> simulated_offset_ns{0};
std::atomic<double>   current_battery_soc{100.0};
```
Đổi các điểm sử dụng thành `.store()` / `.load()` (mặc định `seq_cst` — tần số thấp, không cần tối ưu ordering). Xóa trường chết `last_frame_timestamp_ns` (:87/:177, viết không ai đọc).

**Test mới** (`tests/test_lockfree_c_api.cpp` bổ sung):
```cpp
TEST(CApiConcurrency, TwoReadersOneWriterStress) {
    // writer: push_sensor_frame 1 kHz trong 2s, mỗi frame set sequence_number++
    // reader1: get_uncertainty_state liên tục; reader2: get_sms_payload + get_ble_adv_payload
    // assert: mọi state đọc được có sequence_number thuộc tập đã publish (không giá trị lạ)
    //         + toàn bộ trường số là finite + không crash (chạy ~2000 vòng)
}
TEST(CApiConcurrency, IntegrityStatusIsRaceFree) {
    // luồng A gọi advance_simulated_time liên tục; luồng B gọi get_integrity_status liên tục
    // assert: không crash, is_healthy ∈ {0,1}
}
```
*Lưu ý môi trường: MinGW không có TSAN — test stress trên là.biện pháp thực dụng; nếu sau này build MSVC/Clang thì bật TSAN chạy lại test này.*

---

## NGHIỆM THU TỔNG (Definition of Done)

1. `cmake --build build -j` sạch, không warning mới (khuyến nghị bật `-Werror` cho GCC ở bước này — hiện Rule.md §I.2 yêu cầu nhưng CMakeLists.txt:21 chưa có).
2. `ctest` 100% PASS — kỳ vọng **137 → ~150 test** (13 test regression mới).
3. Mỗi bug 1 commit riêng, message theo mẫu: `fix(scope): <mô tả> [C1|H1..H6] + regression test`.
4. Chạy lại 1 benchmark M0 sau bước 4: `mean_nees_2d` ∈ [0.5, 3.0] (chứng minh chuỗi đo lường đã đáng tin).
5. Smoke desktop GUI sau bước 2 (HUD euler hiển thị đúng chiều nghiêng).

## RỦI RO & VIỆC MỞ LIÊN QUAN (không chặn kế hoạch)

- **H1 làm fail test cũ gắn công thức sai** — bắt buộc sửa expectation; nếu có consumer ngoài (desktop HUD) hiển thị euler, đồng bộ nhãn.
- **Rule.md §I.3 yêu cầu core `-fno-exceptions` nhưng `core/CMakeLists.txt` chưa set** — H4 đã thiết kế theo hướng pre-validation nên an toàn cả hai trường hợp; quyết định bật/tắt cờ để bàn riêng (bật = phải rà toàn bộ core không có đường throw).
- **C1 khắc phục hệ quả nhưng chưa sửa "gốc" Bug 8 residual** (C-API tự chế ngưỡng 15/5% thay vì dùng `gov.target_particle_count`; `fast_loop_rate_hz`/`pf_update_interval_steps` vô consumer) — thuộc gói MEDIUM, làm tiếp sau 7 bug này.
