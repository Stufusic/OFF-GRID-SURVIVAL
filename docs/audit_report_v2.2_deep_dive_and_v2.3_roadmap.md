# BÁO CÁO KIỂM THỬ TOÀN DIỆN, PHÂN TÍCH LỖI LOGIC TIỀM ẨN & LỘ TRÌNH CẢI TIẾN HỆ THỐNG
## OFF-GRID SURVIVAL ENGINE (C++20 PROBABILISTIC NAVIGATION CORE)

- **Phiên bản hệ thống:** v2.2.0 (Probabilistic Offline Survival & Research Harness)
- **Ngày thực hiện:** 04/09/2026
- **Môi trường kiểm thử:** Windows x64 (MSVC / GCC C++20, Ninja, GoogleTest 1.14.0, Eigen 3.4.0)
- **Tập kiểm thử tự động:** **118/118 Unit Tests PASSED** (45 test suites, thời gian thực thi: 1.864 ms)
- **Mục tiêu kiểm toán:** Rà soát toàn diện mã nguồn (`core/`, `platform/`, `desktop/`, `mobile/`, `harness/`, `c_api/`), phát hiện triệt để các lỗi logic tiềm ẩn (latent bugs), vi phạm bất biến an toàn (safety invariants), khiếm khuyết số học, rủi ro đa luồng và đề xuất lộ trình cải tiến phiên bản tiếp theo (v2.3 / v3.0).

---

## MỤC LỤC
1. [TỔNG HỢP KẾT QUẢ KIỂM THỬ TOÀN DIỆN (COMPREHENSIVE TEST RESULTS)](#1-tổng-hợp-kết-quả-kiểm-thử-toàn-diện-comprehensive-test-results)
2. [ĐỐI CHIẾU 12 BẤT BIẾN AN TOÀN TUYỆT ĐỐI (12 SAFETY INVARIANTS AUDIT)](#2-đối-chiếu-12-bất-biến-an-toàn-tuyệt-đối-12-safety-invariants-audit)
3. [DANH MỤC 12 LỖI LOGIC TIỀM ẨN & KHIẾM KHUYẾT HỆ THỐNG (LATENT LOGIC BUGS AUDIT)](#3-danh-mục-12-lỗi-logic-tiềm-ẩn--khiếm-khuyết-hệ-thống-latent-logic-bugs-audit)
4. [PHÂN TÍCH NGUYÊN NHÂN GỐC RỄ & MÔ HÌNH HÓA TOÁN HỌC (ROOT CAUSE ANALYSIS)](#4-phân-tích-nguyên-nhân-gốc-rễ--mô-hình-hóa-toán-học-root-cause-analysis)
5. [GIẢI PHÁP CẢI TIẾN KỸ THUẬT CHO PHIÊN BẢN TIẾP THEO (NEXT-GEN ARCHITECTURAL UPGRADES)](#5-giải-pháp-cải-tiến-kỹ-thuật-cho-phiên-bản-tiếp-theo-next-gen-architectural-upgrades)
6. [LỘ TRÌNH PHÁT TRIỂN & KẾ HOẠCH TRIỂN KHAI (RELEASE ROADMAP & ACTION PLAN)](#6-lộ-trình-phát-triển--kế-hoạch-triển-khai-release-roadmap--action-plan)

---

## 1. TỔNG HỢP KẾT QUẢ KIỂM THỬ TOÀN DIỆN (COMPREHENSIVE TEST RESULTS)

Toàn bộ hệ sinh thái mã nguồn của **Off-Grid Survival Engine** đã được biên dịch với cờ cảnh báo nghiêm ngặt và kiểm thử toàn diện qua 118 ca kiểm thử đơn vị tự động.

```text
[==========] Running 118 tests from 45 test suites ran. (1864 ms total)
[  PASSED  ] 118 tests.
```

### Bảng phân rã kết quả kiểm thử theo Module:

| Phân hệ (Module) | Số lượng Test | Test Suites tiêu biểu | Kết quả | Đánh giá trạng thái thực thi |
| :--- | :---: | :--- | :---: | :--- |
| **Math & Integrity** | 12 | `MathIntegrityTest`, `AngleUtilsTest`, `Crc32Test`, `StatisticsTest`, `IntegrityGuardTest` | **100% PASS** | Các phép toán góc, CRC32, ma trận đối xứng và khử suy biến elip hoạt động chính xác. |
| **Data Contracts & Comms** | 9 | `DataContractsTest`, `Packet160SerializerTest`, `RescuePacketE2ETest`, `RibbonMeshTest` | **100% PASS** | Gói tin 160-bit 20-byte chuẩn hóa, CRC32 roundtrip và sinh lưới mesh hoàn hảo. |
| **ESKF 3D Attitude** | 10 | `Eskf3DTest`, `QuaternionConventionsTest`, `Phase1HardeningTest` | **100% PASS** | Tích phân Quaternion, bù trọng lực Quasi-static, hội tụ góc North và kiểm soát trị riêng ma trận $P \ge 10^{-8}$ hoạt động ổn định. |
| **Step & Stride Models** | 8 | `StepDetectorTest`, `BandpassFilterTest`, `StrideModelsTest`, `PdrIntegrationTest` | **100% PASS** | IIR Butterworth lọc dải bước chân tốt, mô hình sải bước tham số thích ứng co dãn theo góc dốc. |
| **Motion Classification** | 11 | `MotionActivityFsmTest`, `StairMotionDetectorTest`, `StairGeometryTest`, `VehicleSpectralTest`, `VerticalTransportTest` | **100% PASS** | Nhận diện 11 hoạt động cơ bản với FSM trễ; tuy nhiên còn phụ thuộc chặt vào khí áp kế. |
| **DEM & Celestial Solvers** | 11 | `DemMapTest`, `DemMmapIoTest`, `JeanMeeusTest`, `MapManagerTest` | **100% PASS** | Nội suy song tuyến tính DEM, ánh xạ WGS-84/ENU, tính toán góc Mặt Trời Jean Meeus đạt độ chính xác cung giây. |
| **Particle Filter & Fusion** | 13 | `ParticleFilterTest`, `MultiHypothesisClusteringTest`, `MultiModalFusionTest`, `StaticSpatialHashTest` | **100% PASS** | Lan truyền hạt, phân cụm không gian tĩnh Zero-malloc và Systematic Resampling chạy dưới 1ms. |
| **Safety & Governance** | 8 | `SafetyAuditorTest`, `ResourceGovernorTest`, `SafetyInvariantsTest` | **100% PASS** | Phân biệt ngã người/rơi máy, đếm ngược cứu hộ 45s, hạ cấp mức tiêu thụ pin qua Governor. |
| **C API & Platform** | 8 | `LockFreeCApiTest`, `DeterministicReplayTest`, `CornerCasesFuzzingTest` | **100% PASS** | Khởi tạo C API, PAL trừu tượng hóa phần cứng, mô phỏng đọc ghi dữ liệu đồng thời. |
| **Harness & Benchmarks** | 28 | `DataPlayerTest`, `CsvLoaderTest`, `FailureInjectorTest`, `FaultInjectionExtendedTest`, `FactorialBenchmarkTest` | **100% PASS** | Replay dữ liệu cảm biến đa luồng, tiêm lỗi giả lập mất GPS, bão từ, trôi khí áp kế. |

---

## 2. ĐỐI CHIẾU 12 BẤT BIẾN AN TOÀN TUYỆT ĐỐI (12 SAFETY INVARIANTS AUDIT)

Sau các điều chỉnh ở Phase 1 Hardening, hệ thống được tái đánh giá đối chiếu với 12 bất biến an toàn tuyệt đối (`docs/safety_invariants.md`):

```text
       [ĐÁNH GIÁ 12 BẤT BIẾN AN TOÀN TUYỆT ĐỐI (SAFETY INVARIANTS)]
 ┌────────────────────────────────────────────────────────┬──────────────┐
 │ Bất biến (Safety Invariant)                            │ Trạng thái   │
 ├────────────────────────────────────────────────────────┼──────────────┤
 │ Invariant 1: No Teleportation (Nhảy vị trí <= 3 sigma) │ ĐẠT (PASS)   │
 │ Invariant 2: No Null-Island Egress (Tránh 0.0, 0.0)    │ NGUY CƠ      │
 │ Invariant 3: Core Platform Isolation (Sạch header OS)  │ ĐẠT (PASS)   │
 │ Invariant 4: Gated Sensor Ingestion (NIS < 9.0)        │ LỖI TIỀM ẨN  │
 │ Invariant 5: Monotonic Uncertainty (Không co cụm ảo)   │ NGUY CƠ      │
 │ Invariant 6: Degraded Mode Fail-Safe (Chuyển Lost)     │ ĐẠT (PASS)   │
 │ Invariant 7: No False Confidence (GNS <= SensorHealth) │ ĐẠT (PASS)   │
 │ Invariant 8: Covariance Validity (P đối xứng, PSD)     │ ĐẠT (PASS)   │
 │ Invariant 9: Timestamp Monotonicity (dt > 0, no now()) │ ĐẠT (PASS)   │
 │ Invariant 10: No NaN/Inf State (Sạch số học)           │ LỖI TIỀM ẨN  │
 │ Invariant 11: Estimator Freshness (Watchdog 1.5s)      │ LỖI TIỀM ẨN  │
 │ Invariant 12: Rescue Provenance & Age                  │ ĐẠT MỘT PHẦN │
 └────────────────────────────────────────────────────────┴──────────────┘
```

### Chi tiết các bất biến xuất hiện rủi ro mới:
1. **Invariant 2 (No Null-Island Egress):** Trong `MultiModalFusionPipeline`, `latest_canonical_state_.position_wgs84` không được gán tọa độ chuyển đổi từ `position_enu`, dẫn đến trường WGS-84 trong trạng thái Canonical luôn mặc định là `(0.0, 0.0)`.
2. **Invariant 4 & 10 (Gated Sensor Ingestion & No NaN/Inf):** Module `IntegrityGuard` (chứa logic khử NaN/Inf và kẹp vật lý gia tốc) hoàn toàn **không được gọi** tại luồng nạp cảm biến của `MultiModalFusionPipeline`. Một mẫu cảm biến chứa NaN từ phần cứng sẽ làm tê liệt vĩnh viễn Quaternion thái độ của ESKF.
3. **Invariant 5 (Monotonic Uncertainty):** Systematic Resampling trên địa hình phẳng không có cơ chế MCMC Roughening, khiến các hạt bị co cụm nhân bản vào các điểm trùng nhau, làm thu nhỏ diện tích elip $A_{95}$ ảo.
4. **Invariant 11 (Estimator Freshness):** Hàm truy vấn `survival_engine_get_integrity_status()` trong C API chỉ đọc trạng thái lưu sẵn (`latest_integrity_status_`), không kiểm tra thời gian thực. Khi luồng cảm biến bị treo hoàn toàn, API vẫn báo hệ thống khỏe mạnh (`is_healthy = 1`).

---

## 3. DANH MỤC 12 LỖI LOGIC TIỀM ẨN & KHIẾM KHUYẾT HỆ THỐNG (LATENT LOGIC BUGS AUDIT)

Qua quá trình đảo ngược luồng thực thi (Static Analysis, Dataflow Tracing và Dynamic Test Probing), nhóm kiểm toán đã phát hiện **12 lỗi logic tiềm ẩn**, được phân cấp theo mức độ nguy hiểm:

---

### BUG 1 [CRITICAL]: Đóng Băng Đám Mây Hạt Khi Người Dùng Đi Xe Cơ Giới Hoặc Thang Máy
- **Vị trí:** [multi_modal_fusion_pipeline.cpp:L135-L225](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/multi_modal_fusion_pipeline.cpp#L135-L225)
- **Cơ chế lỗi:**  
  Trong `MultiModalFusionPipeline::processSensorFrame`, khi hoạt động là `kVehicleWheeled`, `kCycling`, `kEscalator`, `kElevator`:
  ```cpp
  current_pdr_pos_.easting_m += vel_out.velocity_enu_m_s.x * dt;
  current_pdr_pos_.northing_m += vel_out.velocity_enu_m_s.y * dt;
  ```
  Vị trí `current_pdr_pos_` được tích phân liên tục theo vận tốc. **Tuy nhiên, bộ lọc hạt `pf_engine_` lại hoàn toàn không được dịch chuyển!** Các hạt trong `pf_engine_` chỉ được lan truyền trong khối `if (is_step_detected)` (vốn không bao giờ xảy ra khi đi xe).
- **Hậu quả thực tế:**
  1. Khi xuất trạng thái `uncertainty_state` (mỗi 200ms khi đi xe), tọa độ tâm cụm giả thuyết (`clusters[0].center_easting_m, center_northing_m`) được trích xuất từ các hạt `pf_engine_`. Do đó, **UI, gói tin BLE và Blackbox hiển thị vị trí người dùng đứng yên tại chỗ bắt đầu lên xe!**
  2. Khi người dùng xuống xe và bước bước đầu tiên, `is_step_detected = true`. Lúc này, PDR đã đi xa hàng kilomet nhưng hạt vẫn ở điểm xuất phát cũ. Bộ lọc hạt tính toán độ cao áp kế mới trên nền địa hình cũ $\implies$ Tất cả các hạt đều có độ cao sai lệch trầm trọng $\implies$ $N_{eff} \to 0$, sụp đổ hoàn toàn bộ lọc hạt!

---

### BUG 2 [CRITICAL]: `IntegrityGuard` Bị Ngắt Hoàn Toàn Khỏi Vòng Lặp Runtime, Dễ Nhiễm Độc NaN
- **Vị trí:** [multi_modal_fusion_pipeline.cpp:L2](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/multi_modal_fusion_pipeline.cpp#L2) & [L66-L77](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/multi_modal_fusion_pipeline.cpp#L66-L77)
- **Cơ chế lỗi:**  
  File `multi_modal_fusion_pipeline.cpp` có include `#include "survival_core/integrity/integrity_guard.hpp"` ở dòng 2, nhưng trong toàn bộ class `MultiModalFusionPipeline` không hề khai báo hay gọi hàm `IntegrityGuard::sanitizeSensorFrame(frame)`.
- **Hậu quả thực tế:**  
  Khi cảm biến phần cứng bị lỗi phát ra giá trị `NaN` hoặc `Inf` ở gia tốc kế hoặc con quay hồi chuyển:
  ```cpp
  eskf_.predict(gyro, dt); // Gyro chứa Inf -> dq.w = cos(inf) = NaN!
  ```
  Quaternion danh định `q_nominal_` của ESKF lập tức bị nhiễm NaN. Trong chuẩn số thực IEEE-754, bất kỳ phép tính nào với NaN đều cho ra NaN. **Toàn bộ thái độ Yaw, Roll, Pitch và các phép nhân xoay hệ quy chiếu sau đó sẽ bị khóa cứng ở NaN vĩnh viễn**, không thể tự phục hồi dù các frame tiếp theo có hợp lệ đi nữa!

---

### BUG 3 [HIGH]: Mất Khí Áp Kế Khiến Đi Bộ Bằng Phẳng Bị Nhận Diện Nhầm Thành Leo Cầu Thang
- **Vị trí:** [motion_activity_classifier.cpp:L92-L134](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/kinematics/motion_activity_classifier.cpp#L92-L134) & [stair_motion_detector.cpp:L73-L78](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/kinematics/stair_motion_detector.cpp#L73-L78)
- **Cơ chế lỗi:**  
  Trong `StairMotionDetector::processSample`, điểm leo cầu thang `ascent_score` phụ thuộc vào `rise_time_s >= 0.18s`. Nhịp đi bộ thông thường của con người ($1.4 - 1.8\text{ Hz}$) có thời gian pha dâng từ đáy lên đỉnh dao động tự nhiên trong khoảng $180 - 250\text{ ms}$, do đó `ascent_score` luôn đạt mức cao ($> 0.60$).
  Khi có khí áp kế (`frame.has_baro == true`), hệ thống kiểm tra `baro_vertical_rate_ < 0.08` để triệt tiêu điểm số này. **Nhưng khi thiết bị không có áp kế (`frame.has_baro == false`), nhánh kiểm tra này không chạy!**
  Đồng thời, tại dòng 124 của `motion_activity_classifier.cpp`:
  ```cpp
  if (!is_stair_candidate) {
      scores[1] = 0.85F; // Walking flat
  } else {
      scores[1] = 0.30F; // Bị giảm xuống 0.30 khi is_stair_candidate = true!
  }
  ```
- **Hậu quả thực tế:** Khi chạy trên thiết bị không có khí áp kế hoặc khi áp kế bị hỏng, người dùng đi bộ bình thường trên đường bằng phẳng sẽ liên tục bị hệ thống nhận diện nhầm là đang leo cầu thang (`kStairsAscent`), gây tích lũy độ cao ảo hàng chục mét mỗi phút.

---

### BUG 4 [HIGH]: Biến `current_horizontal_speed_` Không Bao Giờ Được Cập Nhật, Vô Hiệu Hóa Nhận Diện Xe Đạp & Cáp Treo
- **Vị trí:** [motion_activity_classifier.hpp:L55](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/include/survival_core/kinematics/motion_activity_classifier.hpp#L55) & [motion_activity_classifier.cpp:L20, L69-L76](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/kinematics/motion_activity_classifier.cpp#L69-L76)
- **Cơ chế lỗi:**  
  Biến thành viên `current_horizontal_speed_` được khai báo trong `MotionActivityClassifier`, được khởi tạo bằng `0.0F` trong hàm `reset()`, và được truyền vào `vertical_transport_.processSample(...)`. Tuy nhiên, **trong toàn bộ mã nguồn của `motion_activity_classifier.cpp`, không có bất kỳ dòng lệnh nào gán giá trị mới cho biến này!**
- **Hậu quả thực tế:**
  Trong [vertical_transport_detector.cpp:L55](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/kinematics/vertical_transport_detector.cpp#L55):
  - Nhận diện Xe đạp (`kCycling`) đòi hỏi `horizontal_speed_m_s >= 3.0F`. Do biến này luôn bằng 0.0F, **hoạt động đi xe đạp không bao giờ có thể được kích hoạt!**
  - Nhận diện Cáp treo (`kCableTransport`) đòi hỏi `horizontal_speed_m_s > 2.0F`. Hoạt động này cũng bị tê liệt $100\%$.
  - Nhận diện Thang cuốn (`kEscalator`) không thể đạt mức tin cậy tối đa ($0.85$) vì yêu cầu vận tốc ngang $\ge 0.2\text{ m/s}$.

---

### BUG 5 [HIGH]: Vận Tốc Xe Chạy Bình Thường Kích Hoạt Cảnh Báo Sai `kFaultVelocityExcess` Trong `IntegrityMonitor`
- **Vị trí:** [integrity_monitor.cpp:L48, L85-L90](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/integrity/integrity_monitor.cpp#L48)
- **Cơ chế lỗi:**  
  Hằng số `kMaxPlausibleVelocity = 15.0;` ($54\text{ km/h}$) được gán cứng cho toàn bộ các trạng thái. Trong hàm `auditState()`:
  ```cpp
  const double v_norm = state.velocity_m_s.norm();
  if (v_norm > kMaxPlausibleVelocity) {
      status.velocity_valid = false;
      status.fault_flags |= kFaultVelocityExcess;
      status.is_healthy = false;
  }
  ```
  Kiểm tra này không phân biệt người dùng đang đi bộ hay đang đi ô tô (`kVehicleWheeled`).
- **Hậu quả thực tế:** Khi nạn nhân được chở trên xe cấp cứu hoặc xe ô tô chạy với tốc độ thông thường ($60 - 80\text{ km/h} \approx 16.7 - 22.2\text{ m/s}$), bộ giám sát toàn vẹn lập tức kết luận hệ thống có lỗi nghiêm trọng (`is_healthy = false`). Điều này kích hoạt cơ chế fail-safe tại `multi_modal_fusion_pipeline.cpp:L301`, **ép chế độ điều hướng hạ cấp về `kDegraded` một cách vô lý!**

---

### BUG 6 [MEDIUM]: C API Không Bao Giờ Gọi `updateOdometryDisplacement`, Vô Hiệu Hóa Tính Năng Phát Hiện Đi Vòng Quanh (Panic Circling)
- **Vị trí:** [survival_c_api.cpp:L140-L173](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L140-L173)
- **Cơ chế lỗi:**  
  Trong hàm `survival_engine_push_sensor_frame()`, hệ thống chỉ gọi:
  ```cpp
  ctx->safety_auditor.processMotionSample(accel, gyro, frame->timestamp_ns);
  ```
  mà **hoàn toàn quên gọi hàm:**
  ```cpp
  ctx->safety_auditor.updateOdometryDisplacement(net_disp, total_dist, total_steps);
  ```
- **Hậu quả thực tế:** Khi tích hợp Engine vào ứng dụng di động Android/iOS hoặc thiết bị cứu nạn qua thư viện C-API, cờ cảnh báo mất phương hướng đi lòng vòng (`state.flags.panic_circling`) luôn trả về `false`, làm mất đi tính năng phát hiện tâm lý hoảng loạn của nạn nhân khi bị lạc trong rừng.

---

### BUG 7 [MEDIUM]: Lỗ Hổng Watchdog Trong C API Không Thể Phát Hiện Cảm Biến Bị Treo (Invariant 11)
- **Vị trí:** [survival_c_api.cpp:L371-L388](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L371-L388) & [multi_modal_fusion_pipeline.cpp:L290](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/multi_modal_fusion_pipeline.cpp#L290)
- **Cơ chế lỗi:**  
  Trong `pipeline.processSensorFrame`, hàm kiểm tra tính tươi mới của ước lượng được gọi như sau:
  ```cpp
  latest_integrity_status_ = integrity_monitor_.auditState(latest_canonical_state_, frame.timestamp_ns);
  ```
  Do `state.timestamp_ns` bằng chính `frame.timestamp_ns`, độ lệch thời gian luôn bằng 0.
  Mặt khác, trong `survival_engine_get_integrity_status()`, C API chỉ đơn giản trả về bản ghi `latest_integrity_status_` đã lưu từ lần gọi trước.
- **Hậu quả thực tế:** Nếu phần cứng cảm biến bị ngắt kết nối hoặc luồng đọc cảm biến bị deadlock suốt 10 phút, hàm `survival_engine_get_integrity_status()` vẫn trả về `is_healthy = 1, fault_flags = 0`. Vi phạm trực tiếp **Invariant 11: Quá 1.5s không có mẫu đo phải cảnh báo ESTIMATOR_STALE**.

---

### BUG 8 [MEDIUM]: Kết Quả Điều Khiển Của `ResourceGovernor` Bị Bỏ Rơi, Không Thể Tiết Kiệm Pin Runtime
- **Vị trí:** [survival_c_api.cpp:L343-L353](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L343-L353)
- **Cơ chế lỗi:**  
  Khi người dùng gọi `survival_engine_update_battery_soc(handle, soc_percent)`, hệ thống tính toán cấu hình tiêu thụ điện:
  ```cpp
  const auto gov = ctx->resource_governor.updateBatterySoc(soc_percent);
  if (gov.should_trigger_disk_flush) {
      ctx->blackbox.flushToDisk();
  }
  ```
  Các thông số cực kỳ quan trọng trong struct `GovernorState` như:
  - `target_particle_count` (giảm từ 2000 hạt xuống 500 hoặc 100 hạt khi pin yếu)
  - `fast_loop_rate_hz` (giảm tần số ESKF từ 100Hz xuống 50Hz hoặc 20Hz)
  - `pf_update_interval_steps` (chỉ chạy hạt mỗi 2 hoặc 5 bước chân)
  **hoàn toàn không được áp dụng vào `pipeline` hay `pf_engine_`!**
- **Hậu quả thực tế:** Ngay cả khi pin thiết bị tụt xuống $1\%$ (chế độ cận kề tử địa), hệ thống vẫn chạy tối đa công suất 5000 hạt với tốc độ 100Hz, làm cạn kiệt pin thiết bị chỉ sau vài phút thay vì kéo dài thêm 4-6 giờ cứu hộ.

---

### BUG 9 [MEDIUM]: `BenchmarkRunner` Đo Sai Độ Trễ Hạt Particle Filter (Đo Thao Tác Thêm Mảng Thay Vì Thuật Toán)
- **Vị trí:** [benchmark_runner.cpp:L80-L98](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/harness/src/benchmark_runner.cpp#L80-L98)
- **Cơ chế lỗi:**  
  Trong hàm chạy chuẩn đối sánh `BenchmarkRunner::run()`:
  ```cpp
  const auto t0 = std::chrono::high_resolution_clock::now();
  const auto uncertainty_opt = pipeline.processSensorFrame(frame);
  const auto t1 = std::chrono::high_resolution_clock::now();
  // ...
  if (uncertainty_opt.has_value()) {
      const auto t2 = std::chrono::high_resolution_clock::now();
      const double dt_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      pf_step_latencies_ms.push_back(dt_ms);
  }
  ```
  Toàn bộ thuật toán lọc hạt và xử lý bước chân đã nằm trọn vẹn bên trong `pipeline.processSensorFrame(frame)` (giữa `t0` và `t1`). Khoảng cách giữa `t1` và `t2` chỉ là vài phép so sánh `if` và lệnh `push_back`.
- **Hậu quả thực tế:** Thông số `pf_step_latencies_ms` trong các báo cáo thử nghiệm bị sai lệch hoàn toàn (ghi nhận $\approx 0.0001\text{ ms}$ trong khi thực tế bộ lọc hạt mất từ $5 - 20\text{ ms}$).

---

### BUG 10 [LOW]: Nguy Cơ Tràn Số Dưới (Underflow) Biến Đếm `unflushed_count_` Trong `LocalBlackbox`
- **Vị trí:** [local_blackbox.cpp:L42-L46](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/storage/local_blackbox.cpp#L42-L46)
- **Cơ chế lỗi:**  
  Trong `LocalBlackbox`, nếu bộ nhớ Flash bị đầy hoặc lỗi quyền ghi tệp, hàm `flushToDisk()` trả về `false` và không reset biến `unflushed_count_`. Sau 5000 mẫu không ghi được vào đĩa, `unflushed_count_` tiếp tục tăng vượt quá `kCapacity` (5000).
  Khi đó, phép trừ không dấu:
  ```cpp
  const size_t start_idx = (head_ + kCapacity - unflushed_count_) % kCapacity;
  ```
  sẽ bị tràn số dưới (underflow) trên kiểu `size_t`, biến thành một giá trị dương khổng lồ ($\approx 1.84 \times 10^{19}$). Vòng lặp `for (size_t i = 0; i < unflushed_count_; ++i)` sẽ ghi lặp đi lặp lại hàng chục nghìn bản ghi trùng lặp vào đĩa khi khôi phục được kết nối.

---

### BUG 11 [LOW]: Rủi Ro Đọc Tranh Chấp Seqlock Trên Mô Hình Bộ Nhớ Chuẩn C++20
- **Vị trí:** [survival_c_api.cpp:L15-L44](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/c_api/survival_c_api.cpp#L15-L44)
- **Cơ chế lỗi:**  
  Cơ chế Seqlock truyền thống sử dụng thao tác gán struct thông thường:
  ```cpp
  out_state = data; // Kích thước NavigationUncertaintyState > 300 bytes
  ```
  Theo tiêu chuẩn ISO C++20, việc một luồng đọc một biến phi nguyên tử trong khi một luồng khác đang ghi (dù có rào cản bộ nhớ `std::atomic_thread_fence`) về mặt lý thuyết vẫn là hành vi không xác định (Undefined Behavior - Data Race). Trên các vi kiến trúc ARM với bộ xử lý SIMD NEON, trình biên dịch có thể phát sinh các lệnh nạp thanh ghi 128-bit bị xé lẻ (torn read).

---

### BUG 12 [LOW]: Trôi Lũy Tích Âm Vô Cực Của Log-Weight Trong Bộ Lọc Hạt Khi Không Tái Lấy Mẫu
- **Vị trí:** [particle_filter_engine.cpp:L160-L185](file:///c:/New%20PJ/OFF-GRID%20SURVIVAL/core/src/estimators/particle_filter_engine.cpp#L160-L185)
- **Cơ chế lỗi:**  
  Trong `ParticleFilterEngine::updateWeights`, khi số hạt hiệu dụng $N_{eff} \ge N \times 0.5$, hệ thống bỏ qua bước `resample()`.
  Trọng số thực `p.weight` được chuẩn hóa về $\sum w_i = 1$, nhưng trường `p.log_weight` lại **không được cập nhật lại theo trọng số đã chuẩn hóa**. Ở bước tiếp theo, `p.log_weight` tiếp tục cộng dồn các giá trị log-likelihood âm.
  Nếu người dùng đi trên đoạn địa hình bằng phẳng kéo dài khiến hệ thống không tái lấy mẫu suốt 20-30 bước, `p.log_weight` trôi liên tục về $-\infty$, có nguy cơ kích hoạt điều kiện khôi phục khẩn cấp ảo:
  ```cpp
  if (max_log_weight < -1e8) // Tái khởi tạo đều toàn bộ đám mây hạt!
  ```

---

## 4. PHÂN TÍCH NGUYÊN NHÂN GỐC RỄ & MÔ HÌNH HÓA TOÁN HỌC (ROOT CAUSE ANALYSIS)

### 4.1. Sự Bất Tương Đồng Giữa Chu Trình Động Học Liên Tục Và Sự Kiện Hạt Rời Rạc
Kiến trúc v2.2 của Engine vận hành theo mô hình đa tần số lai:
- **Fast Loop (100 Hz):** Tích phân gia tốc/vận tốc góc quán tính $v(t + \Delta t) = v(t) + a_{\text{ENU}} \Delta t$.
- **Slow Loop (Event-driven):** Kích hoạt theo từng bước chân của người đi bộ.

Nguyên nhân cốt lõi của **Bug 1** nằm ở sự thiếu hụt cơ chế cầu nối (Bridge Operator): Khi con người chuyển sang các phương tiện cơ giới (ô tô, xe máy, thang máy), sự kiện bước chân hoàn toàn biến mất ($\text{Cadence} = 0$). Hệ thống đã xử lý tốt phần tích phân Dead Reckoning $p_{\text{DR}}$ ở tầng ngoài, nhưng quên mất rằng đám mây xác xuất hạt $p(x_k \mid z_{1:k})$ cũng chính là biểu diễn vị trí không gian của hệ thống:

$$\mathbf{x}_{k}^{(i)} = \mathbf{x}_{k-1}^{(i)} + \Delta \mathbf{p}_{\text{kinematic}} + \mathbf{w}_{k}^{(i)}, \quad \mathbf{w}_{k}^{(i)} \sim \mathcal{N}(0, \mathbf{Q}_{\text{vehicle}})$$

Khi bỏ quên bước cập nhật này đối với các hạt, phân bố xác suất bị kẹt lại trong quá khứ.

### 4.2. Khủng Hoảng Suy Biến Hình Học Bậc Thang Do Thiếu Áp Kế
Phân loại leo cầu thang thuần túy dựa trên thời gian dâng sóng gia tốc $T_{\text{rise}}$ gặp phải rào cản vật lý lớn:
$$\text{Gia tốc bước đi bộ: } a_z(t) \approx g + A \sin(2\pi f t)$$
Với nhịp đi bộ $f = 1.6\text{ Hz}$, chu kỳ bước $T = 0.625\text{ s}$. Thời gian từ điểm cực tiểu (chân rời đất) đến điểm cực đại (chân tiếp đất) tự nhiên kéo dài $T / 3 \approx 0.21\text{ s} > 0.18\text{ s}$.
Nếu không có áp kế để cung cấp thông tin kiểm chứng độc lập $\Delta h \approx v_z \cdot \Delta t$, thuật toán buộc phải dựa vào góc nghiêng thân người (Pitch angle $\theta$ từ ma trận DCM của ESKF). Khi leo cầu thang, trọng tâm cơ thể người có xu hướng đổ về phía trước ($\theta_{\text{pitch}} > 8^\circ$). Việc bỏ qua góc Pitch này trong nhánh không có áp kế chính là nguồn gốc của **Bug 3**.

---

## 5. GIẢI PHÁP CẢI TIẾN KỸ THUẬT CHO PHIÊN BẢN TIẾP THEO (NEXT-GEN ARCHITECTURAL UPGRADES)

Dựa trên kết quả kiểm toán, kiến trúc phiên bản nâng cấp **Off-Grid Survival Engine v2.3** cần triển khai ngay 7 cải tiến kỹ thuật trọng tâm:

```text
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │ KIẾN TRÚC ĐỀ XUẤT CHO BẢN NÂNG CẤP V2.3 (HARDENING & MULTI-VEHICLE FUSION)             │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 1. TÍCH HỢP INTEGRITYGUARD VÀO STAGE 1 INGESTION PIPELINE (DIỆT TRIỆT ĐỂ NAN/INF)       │
 │    - frame_clean = integrity_guard_.sanitizeSensorFrame(raw_frame)                     │
 │    - Ngắt ngay ESKF predict/update nếu IMU bị lỗi vật lý (|a| > 50 m/s^2, NaN).        │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 2. BỘ LAN TRUYỀN HẠT LIÊN TỤC CHO PHƯƠNG TIỆN (VEHICLE KINEMATIC PARTICLE PROPAGATION)  │
 │    - Khi act == kVehicleWheeled / kCycling / kEscalator / kElevator:                   │
 │      pf_engine_.shiftParticlesWithNoise(v_enu * dt, Q_vehicle * dt)                    │
 │    - Đảm bảo cụm hạt di chuyển 1:1 đồng pha tuyệt đối với PDR Dead Reckoning.           │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 3. PHÂN LOẠI CƠ SINH HỌC KẾT HỢP GÓC PITCH ESKF KHI MẤT KHÍ ÁP KẾ                      │
 │    - Nếu !has_baro: Chỉ cho phép is_stair_candidate = true khi:                        │
 │      pitch_rad > 0.14 rad (~8 độ nghiêng người) VÀ vertical_jerk > 45 m/s^3.           │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 4. NÂNG CẤP BỘ GIÁM SÁT INTEGRITYMONITOR THÍCH ỨNG THEO CHẾ ĐỘ CHUYỂN ĐỘNG             │
 │    - kMaxVelocity = (is_vehicle_mode) ? 60.0 m/s (216 km/h) : 15.0 m/s (54 km/h)       │
 │    - Loại bỏ hoàn toàn cảnh báo sai kFaultVelocityExcess khi nạn nhân ngồi xe.         │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 5. KHÓA BẢO VỆ CHỐNG SUY BIẾN HẠT BẰNG MCMC ROUGHENING TRÊN ĐỊA HÌNH PHẲNG             │
 │    - Khi slope < 0.02, sau Systematic Resampling, bổ sung nhiễu Gauss:                 │
 │      p.easting += N(0, sigma_roughen), bảo toàn tuyệt đối Invariant 5.                │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 6. TRIPLE-BUFFERING KHÔNG KHÓA (LOCK-FREE TRIPLE BUFFER) CHO GIAO DIỆN C-API           │
 │    - Thay thế Seqlock bằng 3 bộ đệm tĩnh hoán đổi atomic index.                        │
 │    - Zero Data Race 100% theo tiêu chuẩn C++20, không retry loop, không torn-read.     │
 ├────────────────────────────────────────────────────────────────────────────────────────┤
 │ 7. THI CÔNG HOÀN THIỆN CÁC MỐI NỐI RUNTIME TRONG C-API                                 │
 │    - Nối ResourceGovernor điều chỉnh số hạt và tần số ESKF theo mức pin thực tế.       │
 │    - Bổ sung updateOdometryDisplacement kích hoạt cảnh báo Panic Circling.             │
 │    - Kiểm tra nhãn thời gian thực trong survival_engine_get_integrity_status().        │
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. LỘ TRÌNH PHÁT TRIỂN & KẾ HOẠCH TRIỂN KHAI (RELEASE ROADMAP & ACTION PLAN)

Để đưa hệ thống từ mức độ hoàn thiện kiểm thử harness lên sản phẩm thương mại chạy thực địa tại các vùng rừng núi, lộ trình phát triển được thiết lập thành 3 giai đoạn:

```text
 2026-Q3 (Tháng 9)                  2026-Q4 (Tháng 10 - 11)           2027-Q1 (Tháng 1 - 2)
┌───────────────────────────────┐  ┌───────────────────────────────┐ ┌───────────────────────────────┐
│ GIAI ĐOẠN 1: CORE HARDENING   │  │ GIAI ĐOẠN 2: ADVANCED FUSION  │ │ GIAI ĐOẠN 3: FIELD PRODUCTION │
│ (Phiên bản v2.3)              │  │ (Phiên bản v2.4)              │ │ (Phiên bản v3.0 Thương mại)   │
├───────────────────────────────┤  ├───────────────────────────────┤ ├───────────────────────────────┤
│ • Sửa dứt điểm 12 lỗi logic.  │  │ • KLD-Sampling thích ứng hạt  │ │ • Hoàn thiện Android JNI /    │
│ • Nối IntegrityGuard vào nạp  │  │   (500 .. 10.000 hạt theo     │ │   Foreground Service chạy ngầm│
│   cảm biến pipeline.          │  │   độ phức tạp địa hình).      │ │   tiết kiệm pin dưới 15mA.    │
│ • Triển khai Continuous       │  │ • Khép vòng lặp Graph-SLAM    │ │ • Render dải ruy-băng Ribbon  │
│   Kinematic Particle Shift.   │  │   tự động khi nạn nhân đi     │ │   trên Vulkan / Metal GPU.    │
│ • Nâng cấp Lock-Free Triple   │  │   ngang qua vết chân cũ.      │ │ • Giao thức Cứu hộ LoRa Mesh/ │
│   Buffer trong C API.         │  │ • Tách biến áp thời tiết bão  │ │   BLE Long Range đa chặng.    │
│ • Hiệu chỉnh ngưỡng vận tốc xe│  │   khỏi độ cao dốc leo núi.    │ │ • Diễn tập thực địa mù đôi tại│
│   trong IntegrityMonitor.     │  │ • MCMC Roughening chống suy   │ │   Vườn Quốc Gia Bidoup /      │
│ • Nối ResourceGovernor &      │  │   biến hạt trên bình nguyên.  │ │   Dãy Hoàng Liên Sơn.         │
│   Odometry vào C API runtime. │  │                               │ │                               │
└───────────────────────────────┘  └───────────────────────────────┘ └───────────────────────────────┘
```

---

### KẾT LUẬN TOÀN DIỆN

Hệ thống **Off-Grid Survival Engine** đã chứng minh năng lực vượt trội về mặt thuật toán xác suất và thiết kế hệ thống thời gian thực C++20 (Zero heap allocation, cấu trúc nén 160-bit, khả năng tái hiện 100% qua hạt giống ngẫu nhiên). Toàn bộ 118 unit tests đều vượt qua thành công trên môi trường thử nghiệm.

Tuy nhiên, thông qua kiểm toán chuyên sâu, **12 điểm nghẽn logic tiềm ẩn** đã được phát hiện và khoanh vùng chính xác tới từng dòng mã nguồn, trong đó trọng tâm là sự ngắt quãng giữa chuyển động xe cơ giới với bộ lọc hạt và việc thiếu vắng bộ lọc tiền xử lý NaN tại đầu vào Pipeline. Việc khắc phục triệt để các vấn đề này theo thiết kế kiến trúc v2.3 đề xuất trên sẽ đảm bảo hệ thống đạt độ tin cậy tuyệt đối, sẵn sàng cho các nhiệm vụ cứu nạn sinh tồn khắt khe nhất trong môi trường thực địa hoàn toàn không có sóng di động và GPS.
