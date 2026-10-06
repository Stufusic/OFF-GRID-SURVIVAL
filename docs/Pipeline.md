# BẢN ĐẶC TẢ CHI TIẾT: PIPELINE KIẾN TRÚC ỨNG DỤNG VÀ LUỒNG DỮ LIỆU VẬN HÀNH

**HỆ THỐNG ĐỊNH VỊ & SINH TỒN NGOẠI TUYẾN (OFF-GRID SURVIVAL ENGINE)**

---

## I. SƠ ĐỒ TỔNG THỂ LUỒNG DỮ LIỆU ĐA TẦNG (END-TO-END ARCHITECTURAL PIPELINE)

Hệ thống vận hành theo mô hình **Đa chu kỳ bất đồng bộ (Multi-Rate Asynchronous Pipeline)**, phân tách tuyệt đối giữa vòng lặp phần cứng thời gian thực, chu trình ước lượng xác suất, và tầng phân phối tài nguyên.

```text
 [CẢM BIẾN PHẦN CỨNG]
   ├── IMU (Acc/Gyro)   [100 Hz] ──► [Lock-Free Ring Buffer 1] ──┐
   ├── Barometer        [ 10 Hz] ──► [Lock-Free Ring Buffer 2] ──┼──► STAGE 1: INGESTION & INTEGRITY GUARD
   ├── Magnetometer     [ 20 Hz] ──► [Lock-Free Ring Buffer 3] ──┤    (NaN/Inf Sanitizer, Allan Noise Model)
   └── Camera / Astro   [Event ] ──► [Async Event Queue]      ──┘                     │
                                                                                      ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │ STAGE 2: FAST LOOP - ESKF ORIENTATION (100 Hz)                                                     │
 │   • Tilt Compensation (DCM Projection)                                                             │
 │   • Quaternion Nominal Kinematics + Error-State Propagation                                        │
 │   • Gyro Bias Tracking & Covariance Inflation Guard                                                │
 └──────────────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                                    │ [Heading ψ, Roll φ, Pitch θ, Covariance P_θ]
                                                    ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │ STAGE 3: MID LOOP - EVENT-DRIVEN PDR PROPAGATION (~1-2 Hz theo nhịp bước)                         │
 │   • Bandpass Filtering (0.5 - 3.0 Hz) + Dynamic Peak/Valley Detection                              │
 │   • Adaptive Parametric Stride: μ_L = f(Cadence, a_v, Slope, L_leg)                                │
 │   • Stochastic Particle Propagation (N = 5000 hạt): x_k^(i) = x_(k-1)^(i) + (L ± σ_L) * (ψ ± σ_ψ)  │
 └──────────────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                                    │ [Đám mây hạt sau dịch chuyển]
                                                    ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │ STAGE 4: SLOW LOOP - MEASUREMENT LIKELIHOOD & RESAMPLING (0.5 - 1 Hz)                              │
 │   • Bilinear DEM Interpolation: z_dem, ∇DEM (Slope, Aspect)                                        │
 │   • Weighted Log-Likelihood Fusion: ln p = λ_elev*ln p_e + λ_slope*ln p_s + λ_asp*ln p_a + ln p_prof│
 │   • Soft Traversability Gate + Celestial Soft Constraint                                           │
 │   • Systematic Resampling khi N_eff < N / 2                                                        │
 └──────────────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                                    │ [Đám mây hạt sau hiệu chỉnh trọng số]
                                                    ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │ STAGE 5: UNCERTAINTY EXTRACTION & GEOMETRY SOLVER                                                  │
 │   • GMM / Spatial Density Clustering ──► Tách K cụm giả thuyết (Hypotheses)                         │
 │   • Eigenvalue Decomposition ──► Bán trục a, b, góc hướng ψ_ell, Diện tích A_95                   │
 │   • Global Navigation Score (GNS) ──► Isotonic Calibration ──► P_calibrated                       │
 └──────────────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                                    │ [NavigationUncertaintyState]
                                                    ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │ STAGE 6: MISSION GOVERNOR & SAFETY AUDITOR                                                         │
 │   • Evidence Fusion: Cú ngã (>3.5G) + Bất động (>3 phút) + Không phản hồi                          │
 │   • Loop / Panic Detection: D_net / S_total < Threshold                                            │
 │   • Dynamic Resource Allocator: Điều tiết tần số f_PF, Radio Duty Cycle theo SoC Pin               │
 └──────────────────┬───────────────────────────────┬────────────────────────────────┬────────────────┘
                    │                               │                                │
                    ▼                               ▼                                ▼
 ┌────────────────────────────────────┐ ┌───────────────────────────────┐ ┌───────────────────────────┐
 │ STAGE 7A: UI & VISUALIZATION       │ │ STAGE 7B: COMMS & SOS EGRESS  │ │ STAGE 7C: LOCAL BLACKBOX  │
 │ • Probability Ribbon Render        │ │ • 160-bit Bit-Packing         │ │ • Persistent Flash Ring   │
 │ • Confidence Ellipse Overlays      │ │ • BLE Non-connectable Beacon  │ │ • Replay Trajectory Dump  │
 │ • Last-Breath E-Ink Framebuffer    │ │ • Opportunistic Burst SMS     │ │ • Telemetry Health Log    │
 └────────────────────────────────────┘ └───────────────────────────────┘ └───────────────────────────┘

```

---

## II. ĐẶC TẢ CHI TIẾT TỪNG GIAI ĐOẠN XỬ LÝ (PIPELINE STAGES)

### Giai đoạn 1: Thu nhận Tín hiệu & Lớp Bảo vệ Toàn vẹn (Ingestion & Integrity Guard)

* **Đầu vào:** Luồng tín hiệu thô từ phần cứng OS (Android Sensor HAL / iOS CoreMotion / Desktop Mock).
* **Cơ chế thực thi:**
1. **Non-blocking Ingestion:** Các luồng cảm biến độc lập ghi dữ liệu vào các hàng đợi vòng đơn nhà sản xuất - đơn người tiêu dùng (Single-Producer Single-Consumer - SPSC Lock-free Ring Buffer).
2. **Sanitization Check:** Loại bỏ lập tức các mẫu chứa giá trị $NaN$, $\pm\infty$, hoặc vượt giới hạn vật lý ($\vert{}\mathbf{a}\vert{} > 50\text{ m/s}^2$, $\Vert{}\mathbf{m}\Vert{} > 200\text{ }\mu\text{T}$).
3. **Timestamp Synchronization:** Chuẩn hóa toàn bộ nhãn thời gian phần cứng về trục thời gian đơn điệu hệ thống ($t_{\text{mono}}$ tính bằng nano-giây), bù trừ trôi dạt đồng hồ (Clock Drift).



---

### Giai đoạn 2: Vòng lặp Nhanh - Ước lượng Hướng & Thái độ (Fast-Loop ESKF 100 Hz)

* **Tần số kích hoạt:** $100\text{ Hz}$ (Cố định theo nhịp dữ liệu Gyroscope).
* **Các bước chuyển đổi dữ liệu:**
1. **Dự đoán Quán tính (Nominal Kinematics):** Tích phân vận tốc góc $\boldsymbol{\omega}_m - \hat{\mathbf{b}}_g$ vào Quaternion trạng thái thái độ $\mathbf{q}$.
2. **Lan truyền Hiệp phương sai:**

$$\mathbf{P}_{\delta\mathbf{x}}(t + \Delta t) = \mathbf{F}_x \mathbf{P}_{\delta\mathbf{x}}(t) \mathbf{F}_x^T + \mathbf{Q}$$


3. **Hiệu chỉnh Đo lường (Khi có mẫu Từ kế):**
* Khử nghiêng vectơ từ trường bằng góc Roll/Pitch hiện tại.
* Tính toán Innovation góc hướng $y = \text{atan2}(\sin(\psi_{\text{mag}} - \hat{\psi}), \cos(\psi_{\text{mag}} - \hat{\psi}))$.
* Đánh giá $R_{\text{adaptive}}$: Nếu gia tốc động $\Delta a > 1.5\text{ m/s}^2$ hoặc biến dạng từ trường $\Delta m > 10\text{ }\mu\text{T}$, tăng $R \to \infty$ để cách ly nhiễu.


4. **Cholesky PSD Enforcement:** Ép ma trận $\mathbf{P}$ đối xứng và kiểm tra trị riêng tối thiểu $\ge 10^{-8}$.



---

### Giai đoạn 3: Vòng lặp Sự kiện - Ước lượng Sải bước & Lan truyền Hạt (PDR Propagation ~1-2 Hz)

* **Cơ chế kích hoạt:** Hướng sự kiện (Event-Driven) - Chỉ chạy khi phát hiện một bước chân hợp lệ.
* **Các bước chuyển đổi dữ liệu:**
1. **Xác thực Bước chân:** Đỉnh gia tốc tổng hợp thỏa mãn $a_{\text{norm}} \ge \text{Threshold}_{\text{dynamic}}$ và khoảng cách thời gian giữa hai đỉnh $0.3\text{s} \le \Delta t_{\text{step}} \le 1.0\text{s}$.
2. **Tính toán Sải bước (Adaptive Parametric Inference):**

$$\mu_L = L_{\text{leg}} \cdot \left[ \alpha \left(\frac{f_{\text{step}}}{f_0}\right) + \beta \left(\frac{\Delta a_v}{a_0}\right)^{1/3} \right] \cdot \Phi(\theta_{\text{slope}})$$


$$\sigma_L = \sigma_0 \cdot \left( 1 + \gamma_1 \vert{}\tan\theta_{\text{slope}}\vert{} + \gamma_2 \frac{\text{Var}(T_{\text{step}})}{\tau_0^2} \right)$$


3. **Lan truyền Đám mây Hạt ($N = 5.000$ particles):**
Với mỗi hạt $i \in [1, N]$:

$$E_k^{(i)} = E_{k-1}^{(i)} + (L_k + \epsilon_L^{(i)}) \sin(\psi_k + \epsilon_\psi^{(i)})$$


$$N_k^{(i)} = N_{k-1}^{(i)} + (L_k + \epsilon_L^{(i)}) \cos(\psi_k + \epsilon_\psi^{(i)})$$



*Nhiễu $\epsilon_L^{(i)} \sim \mathcal{N}(0, \sigma_L^2)$, $\epsilon_\psi^{(i)} \sim \mathcal{N}(0, \mathbf{P}_{\delta\theta_{zz}})$.*



---

### Giai đoạn 4: Vòng lặp Đo lường - Khớp Địa hình & Tái lấy mẫu (Measurement & Resampling 0.5-1 Hz)

* **Tần số kích hoạt:** Mỗi $1 - 2$ bước chân hoặc định kỳ $1.0\text{s}$.
* **Các bước chuyển đổi dữ liệu:**
1. **Truy vấn Bản đồ DEM:** Nội suy song tuyến tính tại tọa độ $(E^{(i)}, N^{(i)})$ để lấy độ cao $z_{\text{dem}}^{(i)}$ và vectơ pháp tuyến bề mặt $\nabla \mathcal{M}_{\text{DEM}}$.
2. **Tính toán Weighted Log-Likelihood (Evidence Tempering):**

$$\ln w_{\text{new}}^{(i)} = \ln w_{\text{old}}^{(i)} + \lambda_{\text{elev}} \ln p_{\text{elev}}^{(i)} + \lambda_{\text{slope}} \ln p_{\text{slope}}^{(i)} + \lambda_{\text{aspect}} \ln p_{\text{aspect}}^{(i)} + \lambda_{\text{prof}} \ln p_{\text{profile}}^{(i)} + \ln p_{\text{travel}}^{(i)}$$


3. **Tích hợp Ràng buộc Thiên văn (Nếu có quan sát Mặt Trời khả dụng):**

$$\ln w_{\text{new}}^{(i)} \leftarrow \ln w_{\text{new}}^{(i)} + \ln p(\mathbf{z}_{\text{sun}} \mid \mathbf{x}^{(i)}, C_{\text{obs}})$$


4. **Chuẩn hóa & Đánh giá $N_{\text{eff}}$:**

$$N_{\text{eff}} = \frac{1}{\sum_{i=1}^N \left(\tilde{w}^{(i)}\right)^2}$$



Nếu $N_{\text{eff}} < \frac{N}{2} \implies$ Thực thi thuật toán **Low-Variance Resampling** trong thời gian $\mathcal{O}(N)$, phân bổ lại trọng số đều nhau $w^{(i)} = \frac{1}{N}$.



---

### Giai đoạn 5: Trích xuất Bất định & Gom cụm Hình học (Uncertainty Extraction)

* **Cơ chế thực thi:**
1. **Phân cụm Không gian:** Áp dụng GMM hoặc DBSCAN với bán kính láng giềng $\epsilon = 150\text{m}$ trên tập hạt sau tái lấy mẫu để tách thành $K$ cụm giả thuyết ($K \le 4$).
2. **Giải tích Hình học Bán trục Ellipse:**
Với mỗi cụm $c \in [1, K]$, tính ma trận hiệp phương sai $2\text{D}$ $\mathbf{\Sigma}_c$:

$$\lambda_1, \lambda_2 = \text{Eigenvalues}(\mathbf{\Sigma}_c) \implies a_{95} = \sqrt{5.9915 \lambda_1}, \quad b_{95} = \sqrt{5.9915 \lambda_2}$$


$$\psi_{\text{ell}} = 90^\circ - \frac{1}{2}\text{atan2}(2\sigma_{EN}, \sigma_E^2 - \sigma_N^2)$$


3. **Định cỡ Xác suất (Isotonic Calibration):**
Tính điểm $GNS = C_{\text{concentration}} \cdot C_{\text{innov}} \cdot C_{\text{map}} \cdot (1 - H_{\text{hyp}})$ và ánh xạ qua bảng tra Isotonic để xuất ra $P_{\text{calibrated}}(\text{Error} \le CE_{95})$.



---

### Giai đoạn 6: Giám sát Sinh tồn & Điều phối Tài nguyên (Mission Governor & Resource Allocator)

* **Cơ chế thực thi:**
1. **Máy trạng thái An toàn (Safety State Machine):**
* `NORMAL` $\to$ Nhận diện cú ngã ($\vert{}\mathbf{a}\vert{} > 3.5\text{G}$) $\to$ Chuyển `PRE_ALERT` (Rung đếm ngược 45s).
* `PRE_ALERT` $\to$ Không có tương tác chạm / Hủy bỏ $\to$ Chuyển `DISTRESS_LOCKED`.
* `NORMAL` $\to$ Kiểm tra quỹ đạo PDR: $D_{\text{net}} < 100\text{m}$ sau $3.000$ bước $\to$ Kích hoạt `PANIC_CIRCLING_WARNING`.


2. **Bộ quản lý Năng lượng Phân đoạn (Discrete Resource Governor):**
* $\text{SoC} > 20\%$: Chế độ `BALANCED` ($f_{\text{PF}} = 10\text{Hz}$, Comms Duty Cycle 5s / 5min).
* $5\% < \text{SoC} \le 20\%$: Chế độ `ENERGY_CONSERVATIVE` ($f_{\text{PF}} = 2\text{Hz}$, Tắt định vị thiên văn camera, lưu Checkpoint mỗi 15 phút).
* $2\% < \text{SoC} \le 5\%$: Chế độ `CRITICAL_SNAPSHOT` (Đóng băng dữ liệu cuối, nạp Outbound Queue, render Bitmap Framebuffer).
* $\text{SoC} \le 2\%$: Chế độ `DISPLAY_ONLY` (Khóa toàn bộ cảm biến, hiển thị màn hình tĩnh cứu nạn).





---

## III. MÔ HÌNH ĐA TIẾN TRÌNH & ĐỒNG THỜI (CONCURRENCY & THREADING MODEL)

Hệ thống phân bổ tài nguyên tính toán thành 3 luồng (Threads) độc lập với mức ưu tiên phần cứng khác nhau, giao tiếp hoàn toàn qua cơ chế phi khóa (Lock-Free) nhằm triệt tiêu hiện tượng tranh chấp (Thread Contention) và bảo vệ giao diện không bị giật lag.

```text
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│ THREAD 1: HARDWARE SENSOR REAL-TIME WORKER (Ưu tiên: Real-Time / SCHED_FIFO)                      │
│   • Lắng nghe Hardware Sensor Callbacks từ Kernel / HAL                                          │
│   • Thực thi Stage 1 (Sanitizer) & Stage 2 (ESKF Fast-Loop 100 Hz)                               │
│   • Ghi State tạm thời vào Double-Buffer                                                         │
└────────────────────────────────────────┬─────────────────────────────────────────────────────────┘
                                         │ Lock-Free SPSC Queue (Step Events + Kinematic State)
                                         ▼
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│ THREAD 2: ESTIMATION & MISSION PIPELINE (Ưu tiên: Normal / Worker Pool)                           │
│   • Thực thi Stage 3 (PDR Propagation)                                                           │
│   • Thực thi Stage 4 (DEM Raycasting & Likelihood Weighting)                                     │
│   • Thực thi Stage 5 (Clustering, Ellipse Geometry, GNS Calibration)                             │
│   • Thực thi Stage 6 (Safety Auditor & Dynamic Energy Governor)                                  │
│   • Ghi NavigationUncertaintyState vào Atomic Shared Snapshot Holder                             │
└────────────────────────────────────────┬─────────────────────────────────────────────────────────┘
                                         │ Atomic Snapshot Read (Zero-Copy)
                                         ▼
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│ THREAD 3: UI & COMMS DISPATCHER (Ưu tiên: Low / Background)                                      │
│   • Render giao diện đồ họa E-Ink / High-contrast UI (Chỉ vẽ khi có snapshot mới)                │
│   • Điều chế dữ liệu 160-bit và quản lý vô tuyến (BLE Advertising Daemon, Burst SMS Queue)       │
│   • Ghi log phân tích xuống tệp nhị phân Flash Storage                                           │
└──────────────────────────────────────────────────────────────────────────────────────────────────┘

```

---

## IV. ĐẶC TẢ GIAO DIỆN DỮ LIỆU ĐẦU VÀO / ĐẦU RA (DATA CONTRACTS)

Toàn bộ các cấu trúc dữ liệu truyền giữa các giai đoạn trong pipeline được định nghĩa tường minh theo chuẩn C++20.

```cpp
#pragma once
#include <cstdint>
#include <array>
#include <span>

namespace survival::pipeline {

// 1. Gói dữ liệu cảm biến đầu vào sau chuẩn hóa
struct alignas(16) SensorFrame {
    uint64_t timestamp_ns{0};
    std::array<float, 3> accel_m_s2{0.0F, 0.0F, 0.0F};
    std::array<float, 3> gyro_rad_s{0.0F, 0.0F, 0.0F};
    std::array<float, 3> mag_uT{0.0F, 0.0F, 0.0F};
    float pressure_hpa{1013.25F};
    float optical_sun_azimuth_rad{-1.0F}; // Giá trị < 0 nếu không có quan sát
    float optical_confidence{0.0F};
};

// 2. Sự kiện bước chân đẩy từ Stage 2 sang Stage 3
struct StepEvent {
    uint64_t timestamp_ns{0};
    double stride_mean_m{0.75};
    double stride_variance_m2{0.0025};
    double heading_rad{0.0};
    double heading_variance_rad2{0.01};
    double vertical_displacement_m{0.0};
    double confidence{1.0};
};

// 3. Hình học bất định của một nhánh giả thuyết
struct HypothesisGeometry {
    uint8_t cluster_id{0};
    double weight{0.0};              // Trọng số phân bố (0.0 - 1.0)
    double center_easting_m{0.0};
    double center_northing_m{0.0};
    double center_altitude_m{0.0};
    
    // Tham số Confidence Ellipse 95%
    double ce95_major_axis_m{0.0};
    double ce95_minor_axis_m{0.0};
    double ellipse_heading_rad{0.0};
    double area_95_m2{0.0};
};

// 4. Trạng thái điều hướng đầy đủ xuất ra UI và Safety Engine
struct NavigationUncertaintyState {
    uint64_t timestamp_ms{0};
    uint32_t session_id{0};
    uint8_t boot_counter{0};
    uint8_t sequence_number{0};
    
    uint8_t active_clusters_count{0};
    std::array<HypothesisGeometry, 4> clusters{};
    
    double global_navigation_score{0.0};
    double calibrated_containment_probability{0.0}; // P(TruePos in CE95)
    
    // Cờ trạng thái hệ thống và sức khỏe cảm biến
    struct SystemFlags {
        bool imu_healthy : 1 {true};
        bool baro_healthy : 1 {true};
        bool mag_anomaly : 1 {false};
        bool celestial_available : 1 {false};
        bool panic_circling : 1 {false};
        bool immobility_warning : 1 {false};
        bool distress_sos_active : 1 {false};
        bool estimator_diverged : 1 {false};
    } flags{};
    
    uint8_t battery_percent{100};
};

// 5. Gói tin nhị phân siêu nén 160-bit (20 Bytes) xuất ra tầng Comms
#pragma pack(push, 1)
struct OutboundRescuePacket160 {
    uint32_t unix_timestamp;     // 32 bits: Thời gian UTC
    int32_t quantized_lat : 24;  // 24 bits: Vĩ độ fixed-point
    uint8_t boot_count;          // 8 bits: Số lần reboot
    int32_t quantized_lon : 24;  // 24 bits: Kinh độ fixed-point
    uint8_t seq_num;             // 8 bits: Số thứ tự gói tin
    uint16_t altitude_m : 12;    // 12 bits: Độ cao (0 - 4095m)
    uint8_t ce50_major_encoded;  // 8 bits: Bán trục lớn CE50
    uint8_t ce50_minor_ratio : 4;// 4 bits: Tỷ số b/a (0 - 15)
    uint8_t ce_heading_encoded;  // 8 bits: Góc hướng trục lớn elip
    uint8_t cluster_weight : 4;  // 4 bits: Trọng số cụm giả thuyết
    uint8_t status_flags;        // 8 bits: Bitfield an toàn & cảm biến
    uint8_t battery_and_subid;   // 8 bits: 5-bit Pin + 3-bit Cluster ID
    uint32_t crc32_checksum;     // 32 bits: Khóa toàn vẹn dữ liệu
};
#pragma pack(pop)
static_assert(sizeof(OutboundRescuePacket160) == 20, "OutboundRescuePacket160 must be exactly 20 bytes!");

} // namespace survival::pipeline

```

---

## V. KỊCH BẢN ĐIỀU PHỐI VẬN HÀNH TRÊN MÁY THỰC ĐỊA (OPERATIONAL WALKTHROUGH)

```text
[TÌNH HUỐNG THỰC ĐỊA: MẤT MẠNG, MÂY MÙ, ĐI VÀO HẺM NÚI SÂU]

1. KHỞI TẠO (T = 0):
   • Thread 1 kích hoạt HAL Sensor, nạp tệp bản đồ địa hình cục bộ "baden_30m.smap" vào RAM.
   • Khởi tạo 5000 hạt phân bố đều quanh điểm mất dấu cuối cùng P_0 (Confidence Ellipse CE95 ban đầu = 200m).

2. DI CHUYỂN DƯỚI TÁN RỪNG (T = 0 -> 2 GIỜ):
   • Thread 1 chạy ESKF 100 Hz giữ góc hướng. Phát hiện bước chân đẩy sang Thread 2.
   • Thread 2 lan truyền hạt. Áp kế báo giảm 120m, đối soát DEM thấy sườn dốc Tây.
   • Tái lấy mẫu giữ lại các hạt nằm đúng khe suối phía Tây.
   • UI Thread hiển thị vệt ruy-băng hẹp (CE95 = 85m, GNS = 0.88, P_calibrated = 91%).

3. GẶP SỰ CỐ NGÃ & BẤT ĐỘNG (T = 2 GIỜ 15 PHÚT):
   • Gia tốc kế ghi nhận xung lực va đập 4.2G, sau đó bất động hoàn toàn.
   • Stage 6 đếm ngược 45s âm thanh/rung cảnh báo. Người dùng bất tỉnh không phản hồi.
   • Mission Governor lập tức:
     - Khóa cứng trạng thái ước lượng cuối cùng (Tránh trôi dạt do hạt phát tán tự do).
     - Gói dữ liệu vào OutboundRescuePacket160 (gồm CE95 major 85m, cờ IMMOBILITY = 1, SOS = 1).
     - Chuyển Wi-Fi Hotspot phát SSID: "SOS_11543N_108234E_ALT820M_BAT74".
     - Kích hoạt BLE Advertising phát xung mỗi 5 giây.
     - Đưa SMS cứu hộ vào hàng đợi ưu tiên cao của Baseband Modem sẵn sàng bắn khi có sóng.

```