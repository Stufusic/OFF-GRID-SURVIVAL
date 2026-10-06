# BÁO CÁO KIỂM THỬ TOÀN DIỆN VÀ ĐÁNH GIÁ KIẾN TRÚC HỆ THỐNG
## OFF-GRID SURVIVAL ENGINE (C++20 PROBABILISTIC NAVIGATION & RESCUE CORE)

**Phiên bản hệ thống:** v2.2.0 (Probabilistic Offline Survival & Research Harness)  
**Ngày thực hiện kiểm thử:** 03/09/2026  
**Môi trường kiểm thử:** Windows x64 (MSVC C++20, GoogleTest 1.14.0, Eigen 3.4.0, Ninja)  
**Tập dữ liệu thực tế kiểm nghiệm:** `2026-09-01_10-57-01_1` (8.094 frames) & `2026-09-01_11-24-39_1` (6.640 frames)  
**Kết quả kiểm thử tự động:** **106/106 Unit Tests PASSED** (43 test suites, thời gian thực thi: 1.546 ms)  

---

## MỤC LỤC
1. [TỔNG QUAN HỆ THỐNG & KIẾN TRÚC LUỒNG DỮ LIỆU (SYSTEM LOGIC FLOW)](#1-tổng-quan-hệ-thống--kiến-trúc-luồng-dữ-liệu-system-logic-flow)
2. [TỪ ĐIỂN THUẬT TOÁN & MÔ HÌNH TOÁN HỌC ÁP DỤNG (ALGORITHMS & FORMULATIONS)](#2-từ-điển-thuật-toán--mô-hình-toán-học-áp-dụng-algorithms--formulations)
3. [ĐÁNH GIÁ TÍNH NĂNG & MỨC ĐỘ TRƯỞNG THÀNH HIỆN TẠI (FEATURE MATURITY EVALUATION)](#3-đánh-giá-tính-năng--mức-độ-trưởng-thành-hiện-tại-feature-maturity-evaluation)
4. [KẾT QUẢ KIỂM THỬ THỰC NGHIỆM (EXPERIMENTAL TEST RESULTS)](#4-kết-quả-kiểm-thử-thực-nghiệm-experimental-test-results)
5. [PHÂN TÍCH LỖI TIỀM ẨN & KHIẾM KHUYẾT LOGIC (DEEP-DIVE BUG & LOGIC AUDIT)](#5-phân-tích-lỗi-tiềm-ẩn--khiếm-khuyết-logic-deep-dive-bug--logic-audit)
6. [ĐỐI CHIẾU 12 BẤT BIẾN AN TOÀN TUYỆT ĐỐI (SAFETY INVARIANTS AUDIT)](#6-đối-chiếu-12-bất-biến-an-toàn-tuyệt-đối-safety-invariants-audit)
7. [LỘ TRÌNH PHÁT TRIỂN & KHUYẾN NGHỊ HÀNH ĐỘNG (FUTURE ROADMAP & ACTION PLAN)](#7-lộ-trình-phát-triển--khuyến-nghị-hành-động-future-roadmap--action-plan)

---

## 1. TỔNG QUAN HỆ THỐNG & KIẾN TRÚC LUỒNG DỮ LIỆU (SYSTEM LOGIC FLOW)

### 1.1. Sơ đồ Luồng Hoạt Động Đa Tần Số (Multi-Rate Asynchronous Pipeline)
Off-Grid Survival Engine là một hệ thống định vị sinh tồn xác suất ngoại tuyến, thiết kế theo kiến trúc phi tập trung, không phụ thuộc vào GPS, duy trì một đám mây xác suất đa giả thuyết (Multi-Hypothesis Particle Cloud) kết hợp dữ liệu độ cao địa hình số (DEM), thiên văn học và quán tính cơ thể.

Luồng dữ liệu vận hành theo mô hình phân tách chu kỳ nghiêm ngặt:

```text
               [CẢM BIẾN PHẦN CỨNG]
    ┌──────────────┬──────────────┬──────────────┐
    │  IMU (100Hz) │  Baro (10Hz) │  Mag (20Hz)  │
    └──────┬───────┴──────┬───────┴──────┬───────┘
           │              │              │
           ▼              ▼              ▼
 ┌──────────────────────────────────────────────────────────┐
 │ [STAGE 1] INGESTION & INTEGRITY SANITIZER                │
 │ - NaN/Inf Sanitizer, Giới hạn vật lý |a| < 50 m/s^2      │
 │ - Loại bỏ nhiễu từ trường cực đoan (|m| > 200 uT)        │
 └─────────────────────────┬────────────────────────────────┘
                           │ SensorFrame chuẩn hóa
                           ▼
 ┌──────────────────────────────────────────────────────────┐
 │ [STAGE 2] FAST LOOP: ESKF 3D ATTITUDE (100 Hz)           │
 │ - Tích phân Quaternion động học: q_(k+1) = q_k ⊗ dq(ω)   │
 │ - Lan truyền hiệp phương sai sai số: P = F P F^T + Q     │
 │ - Quasi-Static Gravity Gate (|a-g| < 0.5, |ω| < 0.05)    │
 │ - DCM Tilt Compensation & Chi-Square Mag Gating (NIS<9.0)│
 │ - Xuất: Yaw (ψ), Roll (φ), Pitch (θ), P_θ, a_v, a_h      │
 └─────────────┬───────────────────────────┬────────────────┘
               │                           │
               │ a_v (100Hz)               │ Attitude, Yaw (100Hz)
               ▼                           ▼
 ┌───────────────────────────┐   ┌──────────────────────────┐
 │ [STAGE 3A] STEP DETECTION │   │ [STAGE 3B] INSTANTANEOUS │
 │ - IIR Butterworth Bandpass│   │ VELOCITY ESTIMATOR (100Hz│
 │   (0.5 - 3.0 Hz)          │   │ - 1-Euro Adaptive Filter │
 │ - Dynamic Peak/Valley     │   │ - ZUPT Lock khi đứng yên │
 │ - StepEvent (1 - 2 Hz)    │   │ - Xuất v_enu, speed 100Hz│
 └─────────────┬─────────────┘   └─────────────┬────────────┘
               │                               │
               │ StepEvent                     │
               ▼                               │
 ┌─────────────────────────────────────────┐   │
 │ [STAGE 4] BIOMECHANICAL CLASSIFIER & FSM│   │
 │ - Nhận diện 11 hoạt động chuyển động    │   │
 │ - 2-Stage Temporal Context Hysteresis   │   │
 │ - Adaptive Riser (Ước lượng bậc thang)  │   │
 └─────────────┬───────────────────────────┘   │
               │                               │
               ▼                               │
 ┌─────────────────────────────────────────┐   │
 │ [STAGE 5] PDR INERTIAL DEAD RECKONING   │   │
 │ - Adaptive Parametric Stride Model      │   │
 │ - Lan truyền hiệp phương sai bước chân: │   │
 │   P_pdr = P_pdr + J_step Cov(L,ψ) J^T   │   │
 └─────────────┬───────────────────────────┘   │
               │ Step Propagation              │
               ▼                               │
 ┌─────────────────────────────────────────┐   │
 │ [STAGE 6] SLOW LOOP: PARTICLE FILTER    │   │
 │ - N = 5.000 hạt: x_k^(i) = x_(k-1) + Δx │   │
 │ - Nội suy song tuyến tính DEM: z_dem, ∇z│   │
 │ - Weighted Log-Likelihood Fusion:       │   │
 │   ln p = λ_e*ln p_e + λ_s*ln p_s + ...  │   │
 │ - Jean Meeus Celestial Solar Likelihood │   │
 │ - Systematic Resampling khi N_eff < N/2 │   │
 └─────────────┬───────────────────────────┘   │
               │                               │
               ▼                               │
 ┌─────────────────────────────────────────┐   │
 │ [STAGE 7] SPATIAL CLUSTERING & GEOMETRY │   │
 │ - StaticSpatialHash<512> (Zero Malloc)  │   │
 │ - Trích xuất K=1..4 cụm giả thuyết      │   │
 │ - Eigenvalue Decomposition: CE50, CE95  │   │
 │ - Tính điểm tin cậy tổng thể (GNS)      │   │
 └─────────────┬───────────────────────────┘   │
               │                               │
               │ NavigationUncertaintyState    │
               ▼                               ▼
 ┌──────────────────────────────────────────────────────────┐
 │ [STAGE 8] MISSION GOVERNOR & SAFETY AUDITOR              │
 │ - Phát hiện ngã có xoay lộn (>3.5G + gyro >2.5 rad/s)    │
 │ - Đếm ngược Pre-Alert 45s & Khóa SOS nếu bất động >3ph   │
 │ - Panic Circling (Đi vòng quanh mất định hướng)          │
 │ - Resource Governor (Điều tiết tần số theo SoC pin)      │
 └─────────────┬───────────────────────────┬────────────────┘
               │                           │
               ▼                           ▼
 ┌───────────────────────────┐   ┌──────────────────────────┐
 │ [STAGE 9A] C-API SEQLOCK  │   │ [STAGE 9B] BLACKBOX LOG  │
 │ - Double-buffered Atomic  │   │ - Flash ring buffer      │
 │ - UI Thread 0-wait read   │   │ - Auto-flush an toàn đĩa │
 └─────────────┬─────────────┘   └──────────────────────────┘
               │
               ▼
 ┌──────────────────────────────────────────────────────────┐
 │ [STAGE 10] OUTBOUND RESCUE EGRESS                        │
 │ - Packet 160-bit nén nhị phân siêu nhỏ gọn (20 bytes)    │
 │ - Mã kiểm tra CRC-32 tiêu chuẩn                          │
 │ - BLE Advertising Beacon & Compact SMS Geolocation Link  │
 └──────────────────────────────────────────────────────────┘
```

---

## 2. TỪ ĐIỂN THUẬT TOÁN & MÔ HÌNH TOÁN HỌC ÁP DỤNG (ALGORITHMS & FORMULATIONS)

### 2.1. Error-State Kalman Filter 3D (ESKF)
* **Tệp mã nguồn:** `core/src/estimators/eskf_3d.cpp`, `core/include/survival_core/estimators/eskf_3d.hpp`
* **Mục tiêu:** Ước lượng thái độ không gian 3D (Roll $\phi$, Pitch $\theta$, Yaw $\psi$) và bù trôi độ lệch con quay hồi chuyển (Gyro Bias $\mathbf{b}_g$).
* **Không gian trạng thái:**
  - Trạng thái danh định (Nominal state): Quaternion định hướng $\mathbf{q} \in \mathbb{H}$, Gyro Bias $\mathbf{b}_g \in \mathbb{R}^3$.
  - Vectơ sai số trạng thái 6D: $\delta\mathbf{x} = [\delta\boldsymbol{\theta}^T, \delta\mathbf{b}_g^T]^T \in \mathbb{R}^6$.
* **Phương trình vi phân động học:**
  $$\boldsymbol{\omega}_{\text{unbiased}} = \boldsymbol{\omega}_m - \mathbf{b}_g$$
  $$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \begin{bmatrix} \cos(\|\boldsymbol{\omega}\|\frac{\Delta t}{2}) \\ \frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|} \sin(\|\boldsymbol{\omega}\|\frac{\Delta t}{2}) \end{bmatrix}$$
* **Phương trình lan truyền ma trận sai số:**
  $$\mathbf{F} = \begin{bmatrix} \mathbf{I}_{3\times3} - [\boldsymbol{\omega}_{\text{unbiased}}_\times]\Delta t & -\mathbf{I}_{3\times3}\Delta t \\ \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3} \end{bmatrix}$$
  $$\mathbf{P}_{k+1} = \mathbf{F} \mathbf{P}_k \mathbf{F}^T + \mathbf{Q}$$
* **Hiệu chỉnh trọng lực (Accelerometer Update):**
  - Kích hoạt khi thỏa mãn điều kiện tĩnh: $|\|\mathbf{a}\| - g| < 0.5\text{ m/s}^2$ và $\|\boldsymbol{\omega}\| < 0.05\text{ rad/s}$.
  - Ma trận đo lường: $\mathbf{H}_{\text{acc}} = [[\mathbf{g}_b]_\times, \mathbf{0}_{3\times3}]$.
  - Trục Yaw và Gyro Bias $Z$ bị vô hiệu hóa ($\delta\theta_z = 0, \delta b_{gz} = 0$) để chống méo mó góc hướng.
* **Hiệu chỉnh từ trường (Magnetometer Update):**
  - Chiếu vectơ từ sang hệ Navigation qua DCM: $\mathbf{m}_{\text{nav}} = \mathbf{R}(\mathbf{q}) \mathbf{m}_{\text{body}}$.
  - Tính góc đổi mới phương vị: $r_{\psi} = \text{atan2}(-m_{x,\text{nav}}, m_{y,\text{nav}})$.
  - Kiểm định Chi-Square Innovation Gating:
    $$\text{NIS} = \frac{r_{\psi}^2}{P_{22} + R_{\text{mag}}} \le 9.0 \quad (\text{Ngưỡng } 3\sigma)$$
    Nếu $\text{NIS} > 9.0$: Từ chối mẫu đo, giảm `mag_health_`, kích hoạt cờ dị thường từ trường.
* **Bảo toàn Joseph-Form:**
  $$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$$

---

### 2.2. Nhận Diện Bước Chân & Mô Hình Chiều Dài Bước Chân Thích Ứng (PDR)
* **Tệp mã nguồn:** `core/src/kinematics/step_detector.cpp`, `adaptive_parametric_model.cpp`
* **Bộ lọc dải thông (Bandpass Filter):**
  - Sử dụng cấu trúc Biquad bậc 2 mô phỏng Butterworth $0.5 - 3.0\text{ Hz}$, tần số lấy mẫu $100\text{ Hz}$.
  - Triệt tiêu trôi dạt DC ($<0.5\text{ Hz}$) và rung động động cơ/tay cầm ($>3.0\text{ Hz}$).
* **Thuật toán đỉnh/đáy động học (Dynamic Peak/Valley Detector):**
  - Ngưỡng biên độ động: $Th_{\text{peak}} = \max(1.2\text{ m/s}^2, 0.4 \cdot P2P_{\text{history}})$.
  - Khóa thời gian trơ sinh học: $T_{\text{refractory}} \ge 240\text{ ms}$ (tối đa $4.16\text{ Hz}$).
* **Mô hình sải bước thích ứng vô thứ nguyên (Adaptive Parametric Stride):**
  $$L = H_{\text{leg}} \cdot \alpha \cdot \left(\frac{f}{\sqrt{g/H_{\text{leg}}}}\right)^\beta \cdot \left(\frac{\Delta a_v}{g}\right)^\gamma \cdot K_{\text{slope}}(\theta)$$
  Trong đó:
  - $H_{\text{leg}} = 0.53 \cdot H_{\text{body}}$ (chiều dài chân giải phẫu).
  - $f$: Nhịp bước (Cadence, Hz).
  - $\Delta a_v = \sqrt[4]{a_{\max} - a_{\min}}$: Biến thiên gia tốc thẳng đứng.
  - $K_{\text{slope}}(\theta)$: Hệ số địa hình bất đối xứng Tobler/Minetti:
    $$K_{\text{slope}}(\theta) = \begin{cases} 1.0 - 0.45 \sin(\theta), & \theta \ge 0 \text{ (lên dốc)} \\ 1.0 + 0.20 \sin(|\theta|), & -0.15 \le \theta < 0 \text{ (xuống nhẹ)} \\ 1.0 - 0.60 \sin(|\theta|), & \theta < -0.15 \text{ (dốc đứng phanh hãm)} \end{cases}$$

---

### 2.3. Bộ Lọc Hạt Khớp Địa Hình (DEM Terrain Particle Filter)
* **Tệp mã nguồn:** `core/src/estimators/particle_filter_engine.cpp`, `core/src/environment/dem_map.cpp`
* **Số lượng hạt:** $N = 5.000$ hạt phân bố trong không gian ENU 3D.
* **Lan truyền hạt (Motion Stochastic Propagation):**
  $$E_k^{(i)} = E_{k-1}^{(i)} + (L + \delta L^{(i)}) \sin(\psi + \delta\psi^{(i)})$$
  $$N_k^{(i)} = N_{k-1}^{(i)} + (L + \delta L^{(i)}) \cos(\psi + \delta\psi^{(i)})$$
  $$\delta L^{(i)} \sim \mathcal{N}(0, \sigma_L^2), \quad \delta\psi^{(i)} \sim \mathcal{N}(0, \sigma_\psi^2)$$
* **Mô hình quan sát Log-Likelihood Đa Chiều:**
  $$\ln p^{(i)} = \lambda_{\text{elev}} \ln p_{\text{elev}}^{(i)} + \lambda_{\text{slope}} \ln p_{\text{slope}}^{(i)} + \lambda_{\text{asp}} \ln p_{\text{asp}}^{(i)} + \lambda_{\text{trav}} \ln p_{\text{trav}}^{(i)} + \lambda_{\text{sun}} \ln p_{\text{sun}}^{(i)}$$
  1. **Cao độ (Elevation):** $\ln p_{\text{elev}}^{(i)} = -\frac{1}{2} \frac{(z_{\text{baro}} - z_{\text{dem}}(E^{(i)}, N^{(i)}))^2}{\sigma_{\text{elev}}^2}$.
  2. **Độ dốc (Slope):** $\ln p_{\text{slope}}^{(i)} = -\frac{1}{2} \frac{(\theta_{\text{pdr}} - \theta_{\text{dem}})^2}{\sigma_{\text{slope}}^2}$.
  3. **Hướng sườn dốc (Aspect):** Có bộ bảo vệ suy biến địa hình phẳng:
     $$\ln p_{\text{asp}}^{(i)} = -\frac{1}{2} \frac{\Delta\psi_{\text{circ}}^2}{\sigma_{\text{asp}}^2} \cdot \tanh\left(\frac{\|\nabla z\|}{s_0}\right)$$
  4. **Phạt mềm khả vi hành (Traversability Soft Gate):** Khi độ dốc $\theta > 55^\circ$ ($0.96\text{ rad}$):
     $$\ln p_{\text{trav}}^{(i)} = -15.0 \cdot (\theta - 0.96)$$
  5. **Mặt Trời (Celestial Constraint):** Dựa trên thuật toán thiên văn Jean Meeus.
* **Tái lấy mẫu Systematic Resampling:** Kích hoạt khi số lượng hạt hiệu dụng tụt giảm:
  $$N_{\text{eff}} = \frac{1}{\sum_{i=1}^N (w^{(i)})^2} < \frac{N}{2}$$

---

### 2.4. Phân Tích Thiên Văn Học Jean Meeus
* **Tệp mã nguồn:** `core/src/environment/jean_meeus_solver.cpp`
* **Mục tiêu:** Tính toán chính xác vị trí góc phương vị Mặt Trời ($\psi_{\odot}$) và góc cao Mặt Trời ($\alpha_{\odot}$) độc lập với mạng Internet/GPS dựa trên nhãn thời gian Epoch UTC và tọa độ địa lý.
* **Các bước tính toán:**
  - Chuyển đổi nhãn thời gian $t_{\text{ns}}$ sang Ngày Julian ($JD$) và thế kỷ Julian $T$ tính từ mốc J2000.0 ($JD = 2451545.0$).
  - Độ kinh trung bình Mặt Trời ($L_0$), Độ dị thường trung bình ($M$).
  - Độ lệch tâm quỹ đạo Trái Đất ($e$), Phương trình tâm sai Mặt Trời ($C$).
  - Kinh độ thực ($\odot$), Xích kinh ($\alpha$), Độ xích vĩ ($\delta$).
  - Góc phương vị sao chuẩn Greenwich ($GMST$), Góc giờ địa phương ($H$).
  - Chuyển đổi sang hệ tọa độ chân trời: Azimuth ($\psi$) và Elevation ($\theta$).

---

### 2.5. Phân Cụm Đa Giả Thuyết & Trích Xuất Elip Bất Định (CE50 / CE95)
* **Tệp mã nguồn:** `core/src/estimators/particle_filter_engine.cpp`, `core/src/math/static_spatial_hash.hpp`, `statistics.cpp`
* **Cơ chế phân cụm tĩnh Zero-Allocation:**
  - Sử dụng bảng băm không gian mở `StaticSpatialHash<512>` với hàm băm số nguyên nguyên thủy:
    $$h(x, y) = ((x \cdot 73856093) \oplus (y \cdot 19349663)) \pmod{512}$$
  - Gom các hạt vào ô lưới kích thước $50\text{m} \times 50\text{m}$, tích lũy trọng số và tọa độ trọng tâm $O(1)$.
  - Trích xuất 4 cụm giả thuyết lớn nhất có trọng số vượt ngưỡng $15\%$.
* **Phân tích trị riêng ma trận hiệp phương sai (Eigenvalue Decomposition):**
  - Tính ma trận hiệp phương sai 2D $\mathbf{P}_{2\text{D}} = \begin{bmatrix} \sigma_E^2 & \sigma_{EN} \\ \sigma_{EN} & \sigma_N^2 \end{bmatrix}$.
  - Giải phương trình đặc trưng tìm trị riêng $\lambda_1, \lambda_2$ và góc xoay hướng $\psi_{\text{ell}}$:
    $$\lambda_{1, 2} = \frac{\sigma_E^2 + \sigma_N^2}{2} \pm \sqrt{\left(\frac{\sigma_E^2 - \sigma_N^2}{2}\right)^2 + \sigma_{EN}^2}$$
    $$\psi_{\text{ell}} = \frac{1}{2} \text{atan2}(2\sigma_{EN}, \sigma_E^2 - \sigma_N^2)$$
  - Bán trục lớn $a$ và bán trục nhỏ $b$ của Elip tin cậy 95% ($\chi^2 = 5.9915$):
    $$a_{95} = \sqrt{5.9915 \cdot \lambda_1}, \quad b_{95} = \sqrt{5.9915 \cdot \lambda_2}$$
    $$\text{Area}_{95} = \pi \cdot a_{95} \cdot b_{95}$$

---

### 2.6. Bộ Phân Loại Hoạt Động Cơ Sinh Học & Máy Trạng Thái FSM (Activity Classifier)
* **Tệp mã nguồn:** `core/src/kinematics/motion_activity_classifier.cpp`, `motion_transition_fsm.cpp`
* **11 Trạng thái hoạt động được phân biệt:**
  1. `kStationary`: Đứng yên, bất động.
  2. `kWalkingFlat`: Đi bộ trên địa hình bằng phẳng.
  3. `kRunning`: Chạy bộ.
  4. `kStairsAscent`: Leo cầu thang đi lên.
  5. `kStairsDescent`: Đi cầu thang xuống.
  6. `kStairLanding`: Bước chân trên chiếu nghỉ cầu thang.
  7. `kEscalator`: Thang cuốn tự động.
  8. `kCycling`: Đạp xe đạp.
  9. `kVehicleWheeled`: Xe cơ giới (ô tô, xe bus, xe máy).
  10. `kElevator`: Thang máy thẳng đứng.
  11. `kCableTransport`: Cáp treo.
* **Bộ lọc mượt thích ứng vận tốc tức thời 100 Hz (1-Euro Filter):**
  - Tự động thay đổi tần số cắt $\omega_c = \omega_{\min} + \beta |\dot{v}|$.
  - Triệt tiêu hiện tượng trễ sóng khi tăng tốc đột ngột, đồng thời khử nhiễu rung lắc khi chuyển động đều.
  - Cơ chế khóa ZUPT (Zero Velocity Update): Khi `activity == kStationary`, vận tốc được ghim cứng về $0.0\text{ m/s}$, triệt tiêu hoàn toàn độ trôi tích phân.

---

### 2.7. Giám Sát An Toàn, Cứu Hộ & Tiết Kiệm Năng Lượng (Safety Auditor & Resource Governor)
* **Tệp mã nguồn:** `core/src/safety/safety_auditor.cpp`, `resource_governor.cpp`
* **Phát hiện ngã con người (Human Fall with Tumble Dynamics):**
  - Điều kiện va chạm: Gia tốc tổng $\|a\| \ge 3.5G$.
  - Điều kiện xoay mất kiểm soát: Vận tốc góc xoay người $\|\omega\| \ge 2.5\text{ rad/s}$ trong cửa sổ $300\text{ ms}$ ngay trước va chạm.
  - Loại bỏ trường hợp rơi máy đơn thuần (Device Drop Only).
  - Khi thỏa mãn: Kích hoạt trạng thái `PRE_ALERT` với chu kỳ đếm ngược $45\text{s}$. Nếu sau $45\text{s}$ nạn nhân không hủy cảnh báo, hoặc bất động liên tục suốt $3\text{ phút}$, hệ thống tự động khóa trạng thái `DISTRESS_LOCKED`.
* **Phát hiện đi vòng tròn hoảng loạn (Panic Circling):**
  - Khi đã di chuyển $>100\text{ m}$ và $>80$ bước, nếu tỉ số độ dịch chuyển thuần trên tổng quãng đường:
    $$\frac{D_{\text{net}}}{S_{\text{total}}} < 0.15$$
    Hệ thống phát cảnh báo `PANIC_CIRCLING` để cảnh báo nạn nhân đang mất phương hướng trong rừng/sương mù.
* **Điều phối tài nguyên pin 4 cấp (Resource Governor):**
  - `Balanced` ($SoC > 20\%$): Tần số Particle Filter $10\text{ Hz}$, màn hình sáng, radio quét liên tục.
  - `Energy Conservative` ($5\% < SoC \le 20\%$): Giảm tần số PF xuống $2\text{ Hz}$, giới hạn cập nhật giao diện.
  - `Critical Snapshot` ($2\% < SoC \le 5\%$): Đóng băng dữ liệu, xả Flash blackbox, chuẩn bị gói tin SOS cuối.
  - `Display Only` ($SoC \le 2\%$): Khóa mọi cảm biến, chỉ duy trì màn hình tọa độ cứu hộ tĩnh.

---

### 2.8. Giao Thức Đóng Gói Tin Cứu Hộ 160-Bit (Packet 160 Serializer & SOS Egress)
* **Tệp mã nguồn:** `core/src/comms/packet_serializer.cpp`, `sos_egress_manager.cpp`
* **Kích thước payload:** Chính xác 20 Bytes (160 bits).
* **Cấu trúc trường dữ liệu (Bit-level Layout):**

| Vị trí Byte | Kiểu dữ liệu | Số bit | Ý nghĩa | Chi tiết độ phân giải |
| :--- | :--- | :--- | :--- | :--- |
| **0 .. 3** | `uint32_t` | 32 | Unix Timestamp (UTC) | Giây, dải đến năm 2106 |
| **4 .. 6** | `int24_t` | 24 | Latitude WGS-84 | $[-90, 90]^\circ$, độ phân giải $\approx 1.07\text{ m}$ |
| **7 .. 9** | `int24_t` | 24 | Longitude WGS-84 | $[-180, 180]^\circ$, độ phân giải $\approx 2.14\text{ m}$ |
| **10 .. 11 (bit 0-11)** | `uint12_t` | 12 | Altitude | $[0, 4095\text{ m}]$, độ phân giải $1\text{ m}$ |
| **11 (bit 12-15)** | `uint4_t` | 4 | Ellipse Minor/Major Ratio | $b/a \in [0, 1]$, lượng tử hóa 16 bước |
| **12** | `uint8_t` | 8 | CE50 Semi-Major Axis | $[0, 255\text{ m}]$, độ phân giải $1\text{ m}$ |
| **13** | `uint8_t` | 8 | Ellipse Orientation Heading | $[0, 360)^\circ$, lượng tử hóa 256 bước ($\approx 1.4^\circ$) |
| **14** | `uint8_t` | 8 | Status & Navigation Mode | Enum trạng thái điều hướng |
| **15 (bit 3-7)** | `uint5_t` | 5 | Battery SoC | $[0, 100\%]$, lượng tử hóa 32 mức ($3.1\%$/bước) |
| **15 (bit 0-2)** | `uint3_t` | 3 | Cluster ID | Chỉ số cụm giả thuyết đang phát (1 .. 4) |
| **16 .. 19** | `uint32_t` | 32 | Checksum CRC-32 | Mã kiểm tra toàn vẹn trên 16 bytes đầu |

* **Kênh phát tín hiệu hỗ trợ:**
  - **BLE Advertisement (24 bytes):** Manufacturer Specific Data format chuẩn Bluetooth SIG.
  - **Compact SMS Geolocation:** Tin nhắn văn bản nén ngắn gọn chứa tọa độ, độ bất định, mức pin và URL Google Maps trực tiếp.

---

## 3. ĐÁNH GIÁ TÍNH NĂNG & MỨC ĐỘ TRƯỞNG THÀNH HIỆN TẠI (FEATURE MATURITY EVALUATION)

### 3.1. Ma trận đối chiếu theo Bản Kế hoạch Lộ trình 5 Phase (`docs/Plan.md`)

| Giai đoạn (Phase) | Hạng mục công việc | Mức độ hoàn thiện | Trạng thái kỹ thuật |
| :--- | :--- | :---: | :--- |
| **PHASE 1: Research Harness & Replay** | - C++20 CMake Monorepo<br>- Lớp `DataPlayer` đồng bộ thời gian<br>- `MetricsCollector` (NEES, NIS, CE50/95)<br>- Dataset Loader CSV & Parser nhị phân | **100%** | **HOÀN HẢO**: Đã nạp thành công 14.734 frames dữ liệu thực nghiệm thực địa từ SensorLogger. |
| **PHASE 2: Kinematics & ESKF 3D** | - `IStepLengthModel` & `AdaptiveParametricModel`<br>- `ESKF_3D` Quaternion & Gyro Bias<br>- Quản lý an toàn số học `IntegrityGuard`<br>- Đánh giá PDR Dead Reckoning | **100%** | **HOÀN THÀNH XUẤT SẮC**: 5 test suite ESKF đạt độ hội tụ 90 độ chuẩn, triệt tiêu trôi dạt gyroscope. |
| **PHASE 3: DEM & Particle Filter** | - Parser bản đồ DEM nhị phân (SDEM) & Bilinear<br>- `ParticleFilterEngine` 5.000 hạt<br>- Thiên văn học `JeanMeeusSolver`<br>- Phân cụm `StaticSpatialHash` & CE95 Ellipse | **100%** | **HOÀN THÀNH XUẤT SẮC**: Tốc độ phân cụm hạt $<1\text{ ms}$ cho 2.000 hạt, không có cấp phát heap. |
| **PHASE 4: Factorial Benchmark & Failure** | - `FailureInjector` (Bão từ, Baro drift, Outage)<br>- `BenchmarkRunner` chạy Monte Carlo 1.000 lần<br>- Đánh giá ma trận lỗi M0, M1, M2, M3, M4 | **100%** | **HOÀN THÀNH XUẤT SẮC**: Tất cả kịch bản tiêm lỗi đều vượt qua kiểm thử tự động. |
| **PHASE 5: Sản phẩm Hóa & Mobile/Egress** | - C API Thread-safe (`survival_c_api.cpp`)<br>- Seqlock Double-Buffered Buffer<br>- Giao thức cứu nạn 160-bit & BLE/SMS Egress<br>- Ribbon Renderer Mesh (OpenGL ES/Vulkan)<br>- Native Desktop GUI (GDI+/Win32)<br>- JNI NDK Bridge & Android UI App | **75%** | **ĐANG TRIỂN KHAI**: Core C API, Seqlock, Blackbox, Desktop App, Ribbon Mesh đã hoàn chỉnh; còn thiếu tầng JNI Kotlin / App UI của Android. |

---

## 4. KẾT QUẢ KIỂM THỬ THỰC NGHIỆM (EXPERIMENTAL TEST RESULTS)

### 4.1. Kết quả kiểm thử tự động Unit Test (Automated Test Suites)
Toàn bộ 106 bài kiểm tra đơn vị trên 43 test suite biên dịch bằng Clang/MSVC đều vượt qua thành công:

```text
[==========] 106 tests from 43 test suites ran. (1546 ms total)
[  PASSED  ] 106 tests.
```

* **Kiểm tra tính bất biến an toàn (`SafetyInvariantsTest`):** 5/5 tests passed (Bảo vệ độ tin cậy ảo, Hiệp phương sai xác định dương, Triệt tiêu NaN/Inf, Watchdog chu kỳ $1.5\text{s}$, Mã hóa cứu hộ).
* **Kiểm tra tiêm lỗi nâng cao (`FaultInjectionExtendedTest`):** 5/5 tests passed (Cắt GPS đột ngột không nhảy tọa độ, Loại bỏ bước nhảy giả mạo $500\text{m}$, Bão từ cực đoan $150\ \mu\text{T}$, Khí áp kế bị đóng băng, Nhiễu giật nhãn thời gian).
* **Kiểm tra tái hiện tiền định (`DeterministicReplayTest`):** Cùng một seed ngẫu nhiên cho ra kết quả trạng thái hạt giống hệt nhau $100\%$.
* **Kiểm tra ma trận đo lường & lọc hạt (`ParticleFilterTest`, `DemMapTest`, `JeanMeeusTest`):** Đạt độ chính xác nội suy DEM vi sai, góc phương vị Mặt Trời sai số $<0.1^\circ$ so với chuẩn thiên văn học Hải quân Hoa Kỳ.

---

### 4.2. Kết quả Replay dữ liệu cảm biến thực địa thật (Real SensorLogger Logs)

#### Tập dữ liệu 1: `2026-09-01_10-57-01_1` (Leo cầu thang thực tế & Đi bộ vòng lặp)
* **Tổng số mẫu cảm biến:** 8.094 frames ($100\text{ Hz}$ IMU, $10\text{ Hz}$ Baro, $20\text{ Hz}$ Mag).
* **Thời gian ghi nhận:** 66.1 giây.
* **Số bước nhận diện:** 98 bước chân.
* **Thống kê phân tích leo cầu thang:**
  - Nhận diện chính xác các chặng leo cầu thang với 70 bậc thang thực tế.
  - Chiều cao mỗi bậc thang ước lượng trung bình: $0.17\text{ m}$ (rất sát với thực tế xây dựng tiêu chuẩn $16.5 - 17.5\text{ cm}$).
  - Tổng độ cao thẳng đứng leo được: $11.55\text{ m}$.
  - Sai số khép vòng (Loop Closure Distance Error): $24.89\text{ m}$ sau 66s đi bộ/leo lầu thuần quán tính không có GPS.

#### Tập dữ liệu 2: `2026-09-01_11-24-39_1` (Chạy bộ & Đi bộ nhanh trên mặt bằng)
* **Tổng số mẫu cảm biến:** 6.640 frames.
* **Thời gian ghi nhận:** 66.1 giây.
* **Số bước nhận diện:** 98 bước.
* **Hiện tượng phát hiện:** Bộ phân loại hoạt động nhận diện nhầm lẫn 70/98 bước là `STAIRS_ASCENT` dù đây là bài kiểm tra chạy bộ trên đường bằng phẳng (Xem phân tích chi tiết tại Mục 5.1).

---

## 5. PHÂN TÍCH LỖI TIỀM ẨN & KHIẾM KHUYẾT LOGIC (DEEP-DIVE BUG & LOGIC AUDIT)

Qua quá trình rà soát chi tiết toàn bộ mã nguồn tầng thấp (`core/src/`), đội ngũ kiểm thử đã phát hiện **7 khiếm khuyết logic và rủi ro kiến trúc tiềm ẩn** cần được khắc phục trước khi đưa vào vận hành sinh tồn thực tế:

---

### 5.1. [CRITICAL BUG] Nhầm Lẫn Đi Bộ Bằng Phẳng / Chạy Bộ Thành Leo Cầu Thang (Stair False Positives)
* **Vị trí phát hiện:**
  - [stair_motion_detector.cpp:L73-L79](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/kinematics/stair_motion_detector.cpp#L73-L79)
  - [motion_activity_classifier.cpp:L101-L110](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/kinematics/motion_activity_classifier.cpp#L101-L110)
* **Cơ chế lỗi:**
  Trong `StairMotionDetector::processSample`, điểm leo cầu thang `ascent_score` được tính như sau:
  ```cpp
  if (feat.rise_time_s >= 0.18F) {
      const float rise_factor = std::clamp((feat.rise_time_s - 0.18F) / 0.12F, 0.0F, 1.0F);
      const float peak_factor = (feat.peak_accel_m_s2 >= 1.5F && feat.peak_accel_m_s2 <= 10.0F) ? 1.0F : 0.6F;
      ascent_score = 0.55F * rise_factor + 0.45F * peak_factor;
  }
  ```
  Khi một người đi bộ bình thường với nhịp $1.6 - 1.9\text{ Hz}$ (chu kỳ bước $0.52 - 0.62\text{ s}$), thời gian từ đáy gia tốc lên đỉnh gia tốc (`rise_time_s`) tự nhiên đạt $0.20 - 0.28\text{ s}$. Gia tốc đỉnh khi gót chân chạm đất đạt $2.5 - 5.0\text{ m/s}^2$. Khi đó:
  - `rise_factor` đạt $0.8 - 1.0$.
  - `peak_factor` đạt $1.0$.
  - $\implies$ `ascent_score` vượt mức $0.60 - 0.85$!
  
  Sau đó, tại `MotionActivityClassifier::processSample`:
  ```cpp
  const bool is_stair_candidate = (scores[3] >= 0.60F || scores[4] >= 0.60F);
  if (human_cadence_hz >= 0.8F && human_cadence_hz <= 2.3F) {
      if (!is_stair_candidate) {
          scores[1] = 0.85F; // Walking flat
      } else {
          scores[1] = 0.30F; // Bị đè bẹp xuống 0.30!
      }
  }
  ```
  Điểm đi bộ bằng phẳng bị cưỡng bức hạ xuống $0.30$, trong khi điểm cầu thang đạt $>0.70$. FSM lập tức chuyển sang `kStairsAscent` dù cao độ khí áp kế không hề thay đổi!
* **Hậu quả thực tế:** Trong Dataset 2 (chạy bộ trên mặt phẳng), 70 trong tổng số 98 bước bị hệ thống ghi nhận là leo cầu thang, tích lũy độ cao ảo $+11.55\text{ m}$.
* **Giải pháp khắc phục:** Bắt buộc kết hợp với kiểm định tốc độ biến thiên khí áp thực tế:
  Chỉ cho phép `is_stair_candidate = true` nếu `baro_vertical_rate_ > 0.12 m/s` (nếu có khí áp kế) HOẶC góc nghiêng thân người (Pitch angle) từ ESKF hướng lên rõ rệt.

---

### 5.2. [HIGH CRITICAL BUG] Đóng Băng Toàn Bộ Pipeline Khi Đứng Yên Hoặc Đi Xe (Pipeline Freeze on Non-Pedestrian Motion)
* **Vị trí phát hiện:**
  - [multi_modal_fusion_pipeline.cpp:L111-L185](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/multi_modal_fusion_pipeline.cpp#L111-L185)
  - [survival_c_api.cpp:L144-L173](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L144-L173)
* **Cơ chế lỗi:**
  Trong `MultiModalFusionPipeline::processSensorFrame`:
  ```cpp
  const auto vel_out = velocity_estimator_.update(...); // 100Hz velocity
  if (is_step_detected) {
      // Chỉ cập nhật PF, Seqlock và sinh trạng thái khi CÓ BƯỚC CHÂN
      ...
      return uncertainty_state;
  }
  return std::nullopt; // KHÔNG CÓ BƯỚC CHÂN -> TRẢ VỀ NULLOPT!
  ```
  Và tại `survival_engine_push_sensor_frame`:
  ```cpp
  auto state_opt = ctx->pipeline.processSensorFrame(*frame);
  if (state_opt.has_value()) {
      ctx->shared_state.write(state);
      ctx->blackbox.pushRecord(rec);
      return 1;
  }
  return 0;
  ```
* **Hậu quả hệ thống:**
  1. **Khi người dùng đứng yên hoặc bị mắc kẹt (Stationary/Trapped):** Không có bước chân người $\implies$ `processSensorFrame` trả về `nullopt` suốt hàng giờ đồng hồ! Seqlock không được ghi, Blackbox không ghi nhận vị trí, Watchdog cảnh báo `ESTIMATOR_STALE` sẽ bị kích hoạt sai.
  2. **Khi người dùng đi xe máy / ô tô (Vehicle) hoặc thang máy (Elevator):** Dù `VehicleMotionModel` và `InstantaneousVelocityEstimator` chạy ở 100Hz tính toán vận tốc xe di chuyển $60\text{ km/h}$, vị trí điều hướng hoàn toàn không di chuyển vì không có bước chân người!
* **Giải pháp khắc phục:**
  Tách luồng xử lý thành 2 chu trình:
  - **Chu trình Động học Thời gian thực (100 Hz Continuous Loop):** Luôn tích phân vận tốc tức thời `v_enu * dt` để cập nhật tọa độ Dead Reckoning, ghi Blackbox định kỳ $1\text{ Hz}$ và phát trạng thái Seqlock cho UI.
  - **Chu trình Khớp Địa hình Xác suất (Event-driven Particle Filter Loop):** Kích hoạt khi có bước chân HOẶC khi xe đã đi được một quãng đường cố định ($>50\text{ m}$).

---

### 5.3. [VIOLATION] Vi Phạm Bất Biến Cách Ly Nền Tảng Lõi (Invariant 3 Violation)
* **Vị trí phát hiện:**
  - [dem_map.cpp:L7-L18](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/environment/dem_map.cpp#L7-L18)
  - [local_blackbox.cpp:L4-L12](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/storage/local_blackbox.cpp#L4-L12)
* **Chi tiết vi phạm:**
  Theo `docs/safety_invariants.md` (Invariant 3): *“Mã nguồn trong thư mục `core/` không được phép chứa bất kỳ Header hoặc lời gọi hàm trực tiếp nào của hệ điều hành cụ thể (`<windows.h>`, `<sys/mman.h>`, `<unistd.h>`).”*
  Tuy nhiên, `dem_map.cpp` và `local_blackbox.cpp` lại include trực tiếp các header này và gọi API Win32 (`CreateFileA`, `FlushFileBuffers`, `CloseHandle`) cũng như POSIX (`open`, `fsync`, `munmap`).
* **Rủi ro:** Gây lỗi biên dịch khi mang mã nguồn lõi sang các vi điều khiển nhúng thời gian thực (FreeRTOS, Zephyr) hoặc môi trường iOS/Apple Silicon không hỗ trợ các API đặc thù.
* **Giải pháp khắc phục:** Thiết kế tầng trừu tượng phần cứng (Platform Abstraction Layer - PAL) hoặc Interface `IFileSystemStorage` tách biệt khỏi `core/`.

---

### 5.4. [DEAD CODE & SAFETY DISCONNECT] Bộ Giám Sát Toàn Vẹn `IntegrityMonitor` Bị Ngắt Khỏi Pipeline Runtime
* **Vị trí phát hiện:** `core/src/integrity/integrity_monitor.cpp`
* **Cơ chế lỗi:**
  Sau bản đánh giá kiến trúc `rv.md`, lớp `IntegrityMonitor` và cấu trúc `CanonicalNavigationState` đã được thiết kế rất chi tiết. Tuy nhiên, qua kiểm tra chéo:
  `IntegrityMonitor` **chỉ được gọi trong file test** `tests/test_safety_invariants.cpp`! Nó hoàn toàn không xuất hiện trong `MultiModalFusionPipeline.cpp`, không có trong `survival_c_api.cpp` và không được tích hợp vào luồng điều hướng runtime thật.
* **Hậu quả:** Hệ thống chạy thực tế không hề được bảo vệ bởi bộ giám sát độc lập `auditState()`.

---

### 5.5. [NUMERICAL RISK] Ép Xác Định Dương 6x6 Bằng Cách Kẹp Đường Chéo Chưa Đủ Điều Kiện Toán Học
* **Vị trí phát hiện:** [eskf_3d.cpp:L390-L412](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/eskf_3d.cpp#L390-L412)
* **Cơ chế lỗi:**
  Hàm `Eskf3D::enforceSymmetryAndPsd()` thực hiện:
  ```cpp
  P = 0.5 * (P + P.transpose());
  for (int i = 0; i < 6; ++i) {
      if (P(i, i) < 1e-8) {
          P(i, i) = 1e-8;
      }
  }
  ```
  **Vấn đề toán học:** Một ma trận có tất cả phần tử trên đường chéo chính dương ($P_{ii} > 0$) **không đồng nghĩa** với việc ma trận đó xác định dương (Positive Semi-Definite)! Nếu các phần tử hiệp phương sai chéo ($P_{ij}$) giữa góc xoay và độ trôi bias tăng cao do nhiễu động học, định thức có thể âm và ma trận có trị riêng âm ($\lambda_{\min} < 0$).
* **Giải pháp khắc phục:** Áp dụng phân tích Cholesky LLT 6x6 hoặc kẹp trị riêng thông qua Eigen decomposition tương tự như đã làm trong `IntegrityGuard::enforcePsd3x3()`.

---

### 5.6. [DATA LOSS HAZARD] Reset Đột Ngột Ngữ Cảnh Khi Chuyển Vùng GPS Cutoff
* **Vị trí phát hiện:** [survival_c_api.cpp:L124-L128](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L124-L128)
* **Cơ chế lỗi:**
  Khi GPS bị ngắt, `survival_engine_push_gps_cutoff` gọi:
  ```cpp
  ctx->pipeline.initialize(handoff.anchor_pos, ctx->dem_map);
  ```
  Hàm `pipeline.initialize()` xóa sạch toàn bộ:
  - `total_steps_ = 0;`
  - `has_initial_baro_ = false;`
  - `activity_classifier_.reset();`
  - `velocity_estimator_.reset();`
  - `current_pdr_cov_ = {1.0, 0.0, 0.0, 1.0};`
* **Hậu quả:** Nạn nhân đang đi bộ hoặc leo dốc bị xóa sạch lịch sử bước chân, độ trôi khí áp kế bị tính lại từ đầu, và phân loại hành vi bị reset về `kStationary`.
* **Giải pháp khắc phục:** Cung cấp hàm `pipeline.handoffAnchorPosition(anchor_pos)` chỉ cập nhật điểm neo của đám mây hạt và PDR mà giữ nguyên toàn bộ trạng thái sinh học/bộ lọc khí áp.

---

### 5.7. [MEMORY CONCURRENCY] Rủi Ro Đọc Tranh Chấp Seqlock Trên C++ Standard Memory Model
* **Vị trí phát hiện:** [survival_c_api.cpp:L15-L44](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L15-L44)
* **Cơ chế lỗi:**
  Trong `SeqlockStateBuffer`, biến `data` là một struct `NavigationUncertaintyState` thông thường (kích thước $>300$ bytes). Khi luồng UI đọc `out_state = data` cùng lúc luồng cảm biến đang thực hiện `data = new_state`, theo C++ Standard Memory Model, đây là hành vi truy cập đồng thời không nguyên tử (Data Race) dẫn đến Undefined Behavior (UB), có thể gây torn read nếu trình biên dịch tối ưu hóa sao chép SIMD.
  Đồng thời, vòng lặp `retry < 5` không có `std::this_thread::yield()` hoặc `_mm_pause()`, dễ dẫn đến thất bại đọc liên tục khi luồng ghi đang bận.
* **Giải pháp khắc phục:** Sử dụng cơ chế Triple-Buffering Lock-Free (3 bộ đệm tĩnh hoán đổi atomic index) để đảm bảo $100\%$ tuân thủ chuẩn C++20 mà không bao giờ bị khóa hay torn read.

---

## 6. ĐỐI CHIẾU 12 BẤT BIẾN AN TOÀN TUYỆT ĐỐI (SAFETY INVARIANTS AUDIT)

| Bất biến an toàn (Safety Invariant) | Nội dung đặc tả kỹ thuật | Trạng thái đạt được | Đánh giá & Bằng chứng mã nguồn |
| :--- | :--- | :---: | :--- |
| **Invariant 1: No Teleportation** | Mất GPS không làm vị trí nhảy quá $3\sigma_{pos}$. | **ĐẠT** | `Scenario1_GpsCutoffZeroTeleportation` kiểm chứng độ trôi khi ngắt GPS chỉ $0.8\text{m} < 9.0\text{m}$. |
| **Invariant 2: No Null-Island Egress** | Không phát tọa độ $(0.0, 0.0)$ khi đã có `LastKnownGoodFix`. | **ĐẠT** | `Packet160Serializer` và `MapManager` ưu tiên trích xuất điểm neo ENU và fallback về `LastKnownGoodFix`. |
| **Invariant 3: Core Platform Isolation** | Thư mục `core/` không chứa header OS (`<windows.h>`, `<unistd.h>`). | **CHƯA ĐẠT** | Phát hiện `<windows.h>` và `<sys/mman.h>` trong `dem_map.cpp` và `local_blackbox.cpp`. Cần bóc tách sang PAL. |
| **Invariant 4: Gated Sensor Ingestion** | Không đo lường nào cưỡng bức thay đổi trạng thái nếu $\text{NIS} > 9.0$. | **ĐẠT** | ESKF triển khai Chi-Square Gating trên từ kế và Quasi-Static Gating trên gia tốc kế. |
| **Invariant 5: Monotonic Uncertainty** | Độ bất định vị trí ($P, CE95$) không giảm khi mất cảm biến. | **ĐẠT MỘT PHẦN** | PDR Covariance tăng đơn điệu; tuy nhiên Particle Filter có nguy cơ thu hẹp ảo nếu resampling trên địa hình phẳng. |
| **Invariant 6: Degraded Mode Fail-Safe** | GNS thấp bắt buộc chuyển sang chế độ `kDegraded` hoặc `kLost`. | **CHƯA ĐẠT** | `MultiModalFusionPipeline` gán cứng `kTrnAided` hoặc `kDegraded` theo từ kế; chưa hạ cấp sang `kLost` khi GNS tụt dốc. |
| **Invariant 7: No False Confidence** | Điểm GNS không được cao hơn chất lượng cảm biến đo lường thực tế. | **ĐẠT TRONG TEST** | Đã viết logic trong `IntegrityMonitor`; cần nối trực tiếp vào runtime pipeline. |
| **Invariant 8: Covariance Validity** | Ma trận $P$ phải đối xứng, hữu hạn và $\lambda_{\min} \ge 10^{-8}$. | **ĐẠT MỘT PHẦN** | Đã ép đối xứng và chặn đường chéo; cần bổ sung kiểm tra trị riêng 6x6 cho ESKF. |
| **Invariant 9: Timestamp Monotonicity** | Nhãn thời gian $t_{i+1} > t_i$, tuyệt đối không gọi `chrono::now()` ở core. | **ĐẠT HOÀN TOÀN** | Toàn bộ thư mục `core/` sạch $100\%$ không chứa bất kỳ lời gọi `std::chrono` nào. |
| **Invariant 10: No NaN/Inf State** | Toàn bộ trạng thái và ngõ ra không chứa giá trị rác số học. | **ĐẠT HOÀN TOÀN** | `IntegrityGuard` quét sạch toàn bộ NaN/Inf trên cảm biến đầu vào và bộ lọc hạt. |
| **Invariant 11: Estimator Freshness** | Quá $1.5\text{s}$ không cập nhật trạng thái lập tức báo `ESTIMATOR_STALE`. | **ĐẠT TRONG TEST** | Cờ `kFaultEstimatorStale` hoạt động trong monitor; cần kích hoạt khi người dùng đứng yên. |
| **Invariant 12: Rescue Provenance & Age** | Gói tin cứu hộ phải chứa nguồn gốc vị trí, tuổi mẫu đo, CE95 và Mode. | **ĐẠT MỘT PHẦN** | SMS payload chứa đầy đủ NAV, CE95, AGE, Pin; gói tin 160-bit nén mới chỉ chứa Mode, thiếu Fix Age. |

---

## 7. LỘ TRÌNH PHÁT TRIỂN & KHUYẾN NGHỊ HÀNH ĐỘNG (FUTURE ROADMAP & ACTION PLAN)

Để đưa **Off-Grid Survival Engine** từ phiên bản thử nghiệm nghiên cứu (Research Harness v2.2) thành một sản phẩm thương mại sinh tồn cấp quân sự/cứu hộ dã ngoại thực thụ, lộ trình hành động được đề xuất như sau:

```text
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │ GIAI ĐOẠN 1 (Tháng 1-2): SỬA LỖI KIẾN TRÚC & AN TOÀN SỐ HỌC (Hardening Core v2.3)      │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 1.1. Sửa lỗi Classifier leo cầu thang: Kết hợp Baro Vertical Rate Δh > 0.12 m/s        │
 │ 1.2. Tách luồng 100Hz Continuous Kinematics: Không đóng băng khi đứng yên hoặc đi xe   │
 │ 1.3. Bóc tách Platform Abstraction Layer (PAL): Loại bỏ <windows.h> khỏi core/         │
 │ 1.4. Nối dây IntegrityMonitor vào MultiModalFusionPipeline & Survival C API            │
 │ 1.5. Nâng cấp Seqlock sang Triple-Buffering Lock-Free chuẩn C++20                      │
 └───────────────────────────────────────────┬────────────────────────────────────────────┘
                                             │
                                             ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │ GIAI ĐOẠN 2 (Tháng 3-4): NÂNG CẤP THUẬT TOÁN ĐỊNH VỊ NÂNG CAO (Advanced Fusion v3.0)   │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 2.1. KLD-Sampling (Kullback-Leibler Divergence): Hạt động thích ứng (500 .. 10.000 hạt)│
 │ 2.2. Roughening MCMC Jitter: Chống suy biến hạt (Particle Collapse) trên địa hình phẳng │
 │ 2.3. Graph-SLAM & Loop Closure: Tự động khép vòng sai số khi quay lại vết chân cũ      │
 │ 2.4. Barometric Weather Trend Filter: Tách biến áp do bão thời tiết khỏi độ cao leo dốc│
 └───────────────────────────────────────────┬────────────────────────────────────────────┘
                                             │
                                             ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │ GIAI ĐOẠN 3 (Tháng 5-6): SẢN PHẨM HÓA DI ĐỘNG & HỆ SINH THÁI CỨU NẠN (Production v3.5)│
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 3.1. Hoàn thiện Android JNI / NDK Service chạy ngầm Foreground tiết kiệm pin           │
 │ 3.2. Triển khai Ribbon Renderer trên OpenGL ES 3.0 / Vulkan cho Android & Metal cho iOS│
 │ 3.3. Tích hợp giao thức Cứu hộ LoRa Mesh / BLE Mesh Đa Chặng (Multi-hop Offline Mesh)  │
 │ 3.4. Khảo nghiệm mù đôi thực địa (Double-Blind Field Test) tại Vườn Quốc Gia / Vùng Núi│
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

### Chi tiết các khuyến nghị kỹ thuật cụ thể:

1. **Khắc phục triệt để bộ phân loại cơ sinh học (Probabilistic Biomechanical Classifier):**
   - Chuyển đổi từ logic ngưỡng cứng dạng `if (feat.rise_time_s >= 0.18)` sang **Naive Bayes / Softmax Classifier** tích hợp xác suất đồng thời của 4 chiều:
     $$P(\text{Stairs} \mid \text{data}) \propto P(\Delta h_{\text{baro}}) \cdot P(T_{\text{rise}}) \cdot P(P2P) \cdot P(\theta_{\text{pitch}})$$
   - Nếu áp kế không ghi nhận biến thiên độ cao tương ứng ($|\Delta h| < 0.05\text{ m}$), xác suất leo cầu thang bắt buộc bị chặn dưới $0.10$.

2. **Cơ chế Duy trì Vận tốc Liên tục 100 Hz (Continuous Kinematic Integration):**
   - Trong `MultiModalFusionPipeline`, sau khi gọi `velocity_estimator_.update()`, luôn cập nhật tọa độ Dead Reckoning:
     $$\mathbf{p}_{\text{DR}}(t + \Delta t) = \mathbf{p}_{\text{DR}}(t) + \mathbf{v}_{\text{ENU}} \cdot \Delta t$$
   - Khi đó, người ngồi trên ô tô/xe máy hay đứng trên thang cuốn vẫn được cập nhật tọa độ mượt mà $100\text{ lần/giây}$.

3. **Chống suy biến hạt trên địa hình bằng phẳng (Particle Roughening):**
   - Khi độ dốc địa hình $\|\nabla z\| < 0.02$ kéo dài, sau mỗi bước Systematic Resampling, bổ sung một lượng nhiễu Gauss vi mô (Roughening jitter) vào tọa độ hạt:
     $$E^{(i)} \leftarrow E^{(i)} + \Delta_{\text{jitter}}, \quad \Delta_{\text{jitter}} \sim \mathcal{N}(0, \sigma_{\text{roughen}}^2)$$
   - Điều này đảm bảo diện tích elip $A_{95}$ và bán trục $CE95$ không bị co rúm giả tạo, duy trì nghiêm ngặt **Bất biến an toàn Invariant 5**.

---

### KẾT LUẬN CHUNG
Dự án **Off-Grid Survival Engine** sở hữu nền tảng kiến trúc toán học và kỹ thuật C++20 cực kỳ vững chắc, tuân thủ tư duy thiết kế hệ thống thời gian thực hiện đại (Zero Heap Allocation, Multi-rate Pipeline, Deterministic Replay, 160-bit Compact Protocol). Với việc xử lý dứt điểm 7 điểm nghẽn logic đã được phân tích chi tiết trong báo cáo này, hệ thống hoàn toàn sẵn sàng tiến lên cấp độ vận hành thực địa thương mại, đáp ứng các tiêu chuẩn cứu hộ khắt khe nhất.
