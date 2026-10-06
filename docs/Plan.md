

# BẢN KẾ HOẠCH DỰ ÁN VÀ ĐẶC TẢ KIẾN TRÚC TỔNG THỂ

**DỰ ÁN: OFF-GRID SURVIVAL ENGINE (Phiên bản 2.2 & Research Harness)**

## 1. TỔNG QUAN DỰ ÁN (PROJECT OVERVIEW)

Dự án xây dựng một hệ thống dẫn đường và sinh tồn xác suất ngoại tuyến (Probabilistic Offline Survival System). Hệ thống không định vị một tọa độ tuyệt đối mà duy trì một **phân phối đa giả thuyết (Multi-Hypothesis Distribution)**, sử dụng kỹ thuật Sensor Fusion kết hợp dữ liệu hình thái địa hình (DEM) và thiên văn.

Trọng tâm giai đoạn hiện tại không phải là phát hành ứng dụng di động ngay, mà là xây dựng **Khung thực nghiệm (Research Harness)** để kiểm chứng, đo lường (NEES, NIS, CE50/95) và tối ưu hóa các thuật toán lõi trên tập dữ liệu thực tế.

---

## 2. KIẾN TRÚC THƯ MỤC DỰ ÁN (MONOREPO STRUCTURE)

Cấu trúc kho lưu trữ (Repository) được thiết kế theo chuẩn C++20 / CMake, phân tách rõ ràng giữa lõi thuật toán, khung kiểm thử và giao diện người dùng.

```text
survival-engine/
├── .github/workflows/             # CI/CD (Tự động chạy Unit Test & Benchmark)
├── CMakeLists.txt                 # File cấu hình build tổng của CMake
│
├── datasets/                      # Tập dữ liệu Ground Truth & Sensor Log
│   ├── metadata.yaml              # Mô tả cảm biến, vị trí, thời tiết
│   ├── train/                     # Dữ liệu để fit mô hình (Isotonic, PDR coeffs)
│   ├── validation/                # Dữ liệu để tuning siêu tham số
│   └── test/                      # Dữ liệu kiểm thử độc lập hoàn toàn
│
├── core/                          # [MODULE 1] LÕI THUẬT TOÁN (C++20 Thuần)
│   ├── include/survival_core/
│   │   ├── estimators/            # ESKF, Particle Filter, GMM Clustering
│   │   ├── kinematics/            # PDR, Bước chân, Thái độ (Attitude)
│   │   ├── environment/           # Parser DEM, Jean Meeus Thiên văn
│   │   └── integrity/             # Cholesky Guard, NaN/Inf Sanitizer
│   └── src/
│
├── harness/                       # [MODULE 2] KHUNG THỰC NGHIỆM ĐẤU TRƯỜNG
│   ├── src/
│   │   ├── DataPlayer.cpp         # Đọc log, đồng bộ thời gian (Time-sync)
│   │   ├── FailureInjector.cpp    # Tiêm nhiễu giả lập (Mất mây, Nhiễu từ)
│   │   ├── MetricsCollector.cpp   # Tính CE50, CE95, NEES, NIS
│   │   └── BenchmarkRunner.cpp    # Chạy Factorial A/B Testing Monte Carlo
│   └── configs/                   # Các file YAML Experiment Manifest
│
├── mobile/                        # [MODULE 3] ỨNG DỤNG NGƯỜI DÙNG (Tương lai)
│   ├── android/                   # Kotlin / JNI cầu nối với lõi C++
│   └── ios/                       # Swift / Objective-C++
│
├── tools/                         # [MODULE 4] CÔNG CỤ XỬ LÝ DỮ LIỆU (Python)
│   ├── allan_variance.py          # Phân tích ARW, Bias Instability từ log tĩnh
│   ├── dem_packager.py            # Nén GeoTIFF thành định dạng Binary cục bộ
│   └── plot_ellipses.py           # Vẽ elip sai số CE95 đè lên bản đồ thực
│
└── docs/                          # Tài liệu kỹ thuật
    ├── MATHEMATICAL_MODELS.md     # Chứng minh công thức toán học
    └── EXPERIMENT_REPORTS/        # Lưu trữ kết quả sau mỗi lần chạy Benchmark

```

---

## 3. PHÂN CHIA MODULE & BỘ TỪ ĐIỂN THUẬT TOÁN BẮT BUỘC

Toàn bộ dự án xoay quanh 5 bài toán hệ thống, mỗi bài toán gắn với các thuật toán đặc thù đã được chuẩn hóa.

### Module 3.1: Ước lượng Động học (Kinematics & PDR)

* **Mục tiêu:** Tính toán bước chân, chiều dài bước và quỹ đạo quán tính cơ sở.
* **Thuật toán & Mô hình:**
* *Step Detection:* Lọc thông dải (Bandpass Filter) $0.5 - 3.0\text{Hz}$ + Peak/Valley Detection.
* *Step Length:* **Mô hình Thích ứng Tham số vô thứ nguyên (Adaptive Parametric)** tích hợp nhịp bước ($f$), biến thiên gia tốc ($a_v$) và góc dốc ($\theta_{\text{slope}}$).
* *Step Uncertainty:* Mô hình tính phương sai $\sigma_L^2$ dựa trên độ nhiễu chu kỳ bước và độ dốc địa hình.



### Module 3.2: Bộ lọc Hướng & Thái độ (Attitude & Heading)

* **Mục tiêu:** Cung cấp góc phương vị (Heading) liên tục, triệt tiêu độ trôi (bias) của Gyroscope.
* **Thuật toán & Mô hình:**
* *Filter Core:* **Error-State Kalman Filter 3D (ESKF)** với vectơ trạng thái sai số 6D $[\delta\theta, \delta b_g]$.
* *Tilt Compensation:* Chiếu ma trận Cosine định hướng (DCM) để khử nghiêng từ trường.
* *Degradation:* Cơ chế **Bơm nhiễu (Covariance Inflation)** kích hoạt Random Walk cho Gyro khi từ trường biến dạng mạnh.



### Module 3.3: Ước lượng Trạng thái Đa Giả thuyết (Multi-Hypothesis Fusion)

* **Mục tiêu:** Hợp nhất PDR, DEM, Thiên văn thành đám mây xác suất; đối phó với địa hình phi tuyến tính.
* **Thuật toán & Mô hình:**
* *Fusion Engine:* **Bộ lọc Hạt (Particle Filter - PF)** với $N = 5.000$ hạt.
* *Resampling:* **Tái lấy mẫu phương sai thấp (Low-Variance / Systematic Resampling)** kích hoạt khi Số lượng hạt hiệu dụng ($N_{\text{eff}}$) tụt quá ngưỡng.
* *Clustering:* **Gaussian Mixture Model (GMM)** hoặc **DBSCAN** để gom cụm đám mây hạt thành các nhánh giả thuyết độc lập.
* *Uncertainty Shape:* Phân tích **Trị riêng (Eigenvalue Decomposition)** ma trận hiệp phương sai để tính bán trục $a, b$ và vẽ Ellipse Định hướng ($CE_{50}, CE_{95}$).



### Module 3.4: Ràng buộc Đo lường & Môi trường (Measurement Models)

* **Mục tiêu:** Tính toán hàm Hợp lý (Likelihood) để cập nhật trọng số cho Particle Filter.
* **Thuật toán & Mô hình:**
* *Terrain Matcher:* Nội suy song tuyến tính (Bilinear Interpolation) ma trận số DEM.
* *Evidence Tempering:* Tích **Weighted Log-Likelihood** (Trọng số phân rã) của 4 yếu tố: Cao độ tuyệt đối, Góc dốc, Hướng sườn dốc (Aspect), Chuỗi cao độ (Profile).
* *Traversability:* Hàm phạt mềm Logistic (Logistic Soft Penalty) khi độ dốc $> 55^\circ$.
* *Astronomy:* Thuật toán **Jean Meeus** tính góc phương vị Mặt Trời + Ray-casting DEM tìm chân trời che khuất.



### Module 3.5: Lớp Toàn vẹn & Hiệu chuẩn (Integrity & Calibration)

* **Mục tiêu:** Đảm bảo hệ thống không sập vì lỗi số học, chuẩn hóa xác suất cho UI.
* **Thuật toán & Mô hình:**
* *Numerical Guard:* Phân tích **Cholesky (LLT/SelfAdjointEigenSolver)** bảo toàn tính Xác định Dương (PSD) của ma trận $\mathbf{P}$.
* *Sensor Noise:* Phân tích **Phương sai Allan (Allan Variance)** tìm ARW và Bias Instability.
* *Confidence:* **Hồi quy đẳng trương (Isotonic Regression)** ánh xạ điểm GNS thành Xác suất thực $P_{\text{loss}}$.



---

## 4. LỘ TRÌNH TRIỂN KHAI THEO PHÂN ĐOẠN (IMPLEMENTATION ROADMAP)

Lộ trình được thiết kế theo tư duy Nghiên cứu Thực nghiệm: Không code giao diện di động cho đến khi chứng minh được thuật toán bằng số liệu trên Desktop.

### PHASE 1: Khởi tạo Dữ liệu & Xây dựng Khung Thực nghiệm (Research Harness)
* **Mục tiêu:** Có môi trường để "chạy lại" (replay) dữ liệu.
* **Hạng mục công việc:**
1. Cấu trúc C++ CMake Monorepo.
2. Viết class `DataPlayer`: Đọc file CSV/Binary log từ điện thoại, đồng bộ mốc thời gian.
3. Viết `MetricsCollector`: Các hàm tính NEES, NIS, $CE_{50}$, $CE_{95}$ so với Ground Truth.
4. Thu thập bộ Dataset thực tế 1 (Ground Truth bằng thiết bị GPS Garmin/RTK). Thực hiện đo Allan Variance.



### PHASE 2: Xây dựng Lõi Động học & Điều hướng (Kinematics & ESKF)

* **Mục tiêu:** Xây dựng hệ thống Inertial/PDR cơ sở.
* **Hạng mục công việc:**
1. Lập trình giao diện `IStepLengthModel` và lớp `AdaptiveParametricModel`.
2. Lập trình `ESKF_3D`: Quản lý Quaternion và Gyro Bias.
3. Tích hợp lớp `IntegrityGuard` (Kiểm tra NaN/Inf, Cholesky Matrix Repair).
4. Cắm vào Harness: Đánh giá độ trôi PDR thô (Chưa có bản đồ).



### PHASE 3: Hệ thống Hợp nhất Đa Giả thuyết (DEM & Particle Filter)

* **Mục tiêu:** Khoanh vùng vị trí bằng bản đồ địa hình và thiên văn.
* **Hạng mục công việc:**
1. Viết bộ Parser ma trận DEM nhị phân cục bộ. Cài đặt các hàm toán tử Gradient tính độ dốc.
2. Lập trình `ParticleFilterEngine`: Propagation, Log-Likelihood Weighting, Low-Variance Resampling.
3. Lập trình module Thiên văn `JeanMeeusSolver`.
4. Viết module Gom cụm (Clustering) xuất ra `NavigationUncertaintyState` (các Ellipse CE95).



### PHASE 4: Đấu trường Thuật toán & Phân tích Đa yếu tố (Factorial Benchmark)
* **Mục tiêu:** Chứng minh tính hiệu quả của các Module.
* **Hạng mục công việc:**
1. Viết `FailureInjector`: Tiêm nhiễu từ trường, trôi áp kế vào luồng dữ liệu của Harness.
2. Viết kịch bản `BenchmarkRunner`: Chạy ma trận Factorial ($2 \times 2 \times 2$) với 1.000 lần Monte Carlo.
3. Xuất báo cáo khoa học (Thống kê $P_{\text{loss}}$, NEES, $CE_{95}$ trên các cấu hình). Chốt các siêu tham số (Hyper-parameters) tốt nhất.



### PHASE 5: Sản phẩm Hóa (Mobile Integration & Caddy Protocol)

* **Thời gian dự kiến:** Bắt đầu sau khi Phase 4 chứng minh thuật toán khả thi.
* **Mục tiêu:** Đưa lõi C++ lên thiết bị thật để chạy Real-time.
* **Hạng mục công việc:**
1. Xây dựng JNI / NDK wrapper cho Android.
2. Tích hợp `ResourceGovernor`: Điều phối tần số PF theo phần trăm pin.
3. Lập trình Giao thức `Paged Multi-Hypothesis Payload` (160-bit nén) + BLE/Wi-Fi Emitter.
4. Khảo nghiệm thực địa mù đôi (Double-Blind Field Test).



---

## 5. MA TRẬN THÍ NGHIỆM ĐÁNH GIÁ (EXPERIMENT MATRIX)

Để chứng minh năng lực hệ thống, các kịch bản sau sẽ được chạy tự động bằng `BenchmarkRunner`:

| Kịch Bản Lỗi (Scenario) | Hiện Tượng Mô Phỏng (Injection) | Metric Bắt Buộc Đánh Giá |
| --- | --- | --- |
| **M0: Nominal (Chuẩn)** | Dữ liệu sạch, nhiễu nền | $CE_{95}$, CPU time |
| **M1: Magnetic Spike** | Tăng vọt từ trường $+45\mu\text{T}$ trong 2 phút | NIS (loại bỏ nhiễu), NEES (giữ hiệp phương sai ổn định) |
| **M2: Baro Drift** | Trừ $2\text{ hPa}$ mỗi giờ (Bão tới) | $P_{\text{loss}}$ (Đảm bảo *Profile Likelihood* cứu được hệ thống) |
| **M3: GPS Blackout** | Đi vào hẻm vực/hang động | Convergence Time (Thời gian bộ lọc hội tụ lại khi ra khỏi hang) |
| **M4: PDR Anomaly** | Randomize phương sai $\sigma_L$ | Tỷ lệ báo động giả (False SOS Rate) |

*Tài liệu này đóng vai trò là kim chỉ nam Kỹ thuật & Thực nghiệm chính thức cho toàn bộ dự án Off-Grid Survival Engine v2.2.*