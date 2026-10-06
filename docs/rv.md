Tôi đã xem bản nâng cấp v2.2. So với bản trước, nó đã tiến khá xa: đã có **frame convention, 6 safety invariants, canonical state, fault injection, emergency override, zero-allocation và E2E rescue validation**. Đây đã bắt đầu giống một architecture document thực thụ chứ không còn là danh sách sửa bug nữa. 

Tuy vậy, tôi vẫn thấy **một số khoảng trống quan trọng**. Tôi sẽ ưu tiên các nâng cấp có khả năng làm hệ thống “khó chết” hơn thay vì chỉ thêm tính năng.

# 1. Nâng cấp lớn nhất: Integrity Monitor độc lập

Hiện tại pipeline tự ước lượng và tự đánh giá confidence của chính nó. Tôi khuyên thêm:

```text
                Navigation Engine
                      │
                      ▼
               Canonical State
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
   Safety Estimator       Integrity Monitor
                                  │
                                  ▼
                         Safety Decision
```

`IntegrityMonitor` không được sửa state. Nó chỉ trả:

```cpp
struct IntegrityStatus {
    bool position_valid;
    bool attitude_valid;
    bool velocity_valid;
    bool covariance_valid;

    double consistency_score;
    FaultCode fault;
};
```

Nó kiểm tra những thứ như:

$$
\|v\| < V_{max}
$$

$$
\|a\| < A_{max}
$$

$$
\|P-P^T\| < \epsilon
$$

$$
\lambda_{min}(P)>0
$$

và đặc biệt:

$$
P \neq NaN,\infty
$$

Nếu estimator bị lỗi nhưng vẫn trả confidence cao, Integrity Monitor phải phát hiện.

**Đây là lớp mà bản v2.2 hiện chưa có.**

---

# 2. Thêm Covariance Health Monitor

Invariant hiện tại yêu cầu uncertainty không giảm giả tạo khi dropout. 

Tôi đề xuất đi xa hơn:

```text
Covariance
   ↓
Symmetry check
   ↓
Positive-semidefinite check
   ↓
Condition number
   ↓
Growth-rate check
```

Kiểm tra:

$$
P \approx P^T
$$

và:

$$
\lambda_i(P)\ge0
$$

Nếu covariance trở nên không hợp lệ:

```text
ESTIMATOR_UNHEALTHY
        ↓
DEGRADED
        ↓
EMERGENCY
```

Điều này đặc biệt hữu ích khi EKF/PF chạy lâu.

---

# 3. Thêm Sensor Health Score riêng cho từng cảm biến

Hiện tại có innovation gating, rất tốt. Nhưng gate chỉ trả lời:

> “measurement này có hợp lệ không?”

Nên thêm:

> “sensor này đang đáng tin đến mức nào?”

Ví dụ:

```cpp
struct SensorHealth {
    float imu;
    float gyro;
    float accel;
    float magnetometer;
    float barometer;
    float gps;
};
```

và:

$$
H_{sensor}\in[0,1]
$$

Health có thể giảm dần khi:

* rejection liên tục
* timestamp lỗi
* noise tăng
* bias tăng
* dropout
* saturation.

Sau đó ESKF/PF có thể dùng:

$$
R_{effective}=\frac{R}{H+\epsilon}
$$

Sensor kém tin → measurement covariance tăng → ảnh hưởng nhỏ hơn.

Cái này tốt hơn kiểu binary:

```text
MAG_VALID = true/false
```

---

# 4. Thêm Bias Estimation + Bias Health

Trong PDR, sai số nguy hiểm nhất không phải noise ngẫu nhiên mà là **bias tích lũy**.

Tôi khuyên state ESKF/PDR có ít nhất:

$$
b_g=(b_{gx},b_{gy},b_{gz})
$$

$$
b_a=(b_{ax},b_{ay},b_{az})
$$

và theo dõi:

$$
\|\dot b_g\|
$$

$$
\|\dot b_a\|
$$

Nếu bias thay đổi bất thường:

```text
BIAS_ANOMALY
```

→ tăng uncertainty.

---

# 5. Thêm Zero-Velocity Update (ZUPT)

Nếu hệ thống nhận diện bước chân thì đây là nâng cấp rất đáng tiền.

Trong pha chân đứng yên:

$$
v \approx 0
$$

cho phép tạo measurement:

$$
z_v=
\begin{bmatrix}
0\\0\\0
\end{bmatrix}
$$

và cập nhật ESKF.

Pipeline:

```text
IMU
 ↓
Step detector
 ↓
Foot/Body stationary detection
 ↓
ZUPT
 ↓
ESKF correction
```

ZUPT có thể giúp giảm drift vận tốc/PDR đáng kể trong các trạng thái dừng hoặc giữa bước, miễn detector đứng yên đủ tin cậy.

---

# 6. Thêm Zero-Angular-Rate Update (ZARU)

Khi thiết bị đứng yên:

$$
\omega\approx0
$$

có thể dùng để ước lượng gyro bias.

```text
Stationary detector
      ↓
gyro ≈ constant
      ↓
estimate bg
      ↓
ESKF correction
```

Kết hợp:

```text
ZUPT + ZARU
```

sẽ mạnh hơn đáng kể cho PDR.

---

# 7. Nâng cấp GPS gating thành "Trajectory Consistency Gate"

Bản v2.2 đã có test GPS jump 500 m và chi-square gating. 

Nhưng GPS jump không phải vấn đề duy nhất.

Có thể xảy ra:

```text
GPS A
 ↓
GPS B  +30m
 ↓
GPS C  +30m
 ↓
GPS D  +30m
```

Mỗi bước riêng lẻ có thể không vượt gate, nhưng cả trajectory phi vật lý.

Vì vậy cần thêm:

$$
v_{GPS}=\frac{\Delta p}{\Delta t}
$$

và:

$$
a_{GPS}=\frac{\Delta v}{\Delta t}
$$

rồi kiểm tra:

```text
position innovation
+
velocity plausibility
+
acceleration plausibility
+
trajectory continuity
```

Đây là **Temporal GPS Integrity Check**.

---

# 8. DEM cũng cần Integrity Layer

Đây là một điểm tôi nghĩ tài liệu hiện tại còn bỏ sót.

Nếu TRN dựa vào DEM, DEM có thể:

* thiếu dữ liệu
* sai độ cao
* có void
* resolution quá thấp
* artifact
* tile boundary discontinuity.

Nên mỗi DEM query trả:

```cpp
struct TerrainSample {
    double elevation_m;
    double slope;
    double curvature;

    float confidence;
    bool valid;
};
```

Không nên chỉ:

```cpp
elevation = dem(x,y);
```

Sau đó particle filter có thể giảm trọng số của terrain measurement khi:

$$
C_{DEM}\downarrow
$$

---

# 9. Thêm DEM Tile Boundary Continuity Test

Một lỗi rất khó thấy:

```text
DEM tile A | DEM tile B
            ↑
         37m jump
```

Particle filter có thể tưởng rằng đây là địa hình thật.

Test nên kiểm tra:

$$
|\Delta h| < H_{max}
$$

ở biên tile nếu địa hình yêu cầu liên tục.

---

# 10. Thêm Map/Particle Filter "Recovery Mode"

Nếu:

$$
N_{eff}
$$

của particle filter tụt thấp:

$$
N_{eff}
=
\frac{1}{\sum w_i^2}
$$

thì có nguy cơ particle collapse.

Đề xuất:

```text
N_eff normal
     ↓
low
     ↓
RESAMPLE
     ↓
still low?
     ↓
REJUVENATION
     ↓
spread particles
```

Ví dụ:

```text
Particle cloud
      ↓
detect collapse
      ↓
inflate covariance
      ↓
reseed around valid region
```

Đây là một nâng cấp rất đáng làm cho PF.

---

# 11. Adaptive Particle Distribution tốt hơn chỉ Adaptive Particle Count

Bản v2.2 hiện đề cập điều chỉnh số hạt theo tài nguyên và emergency override. 

Tôi đề xuất thêm:

> không chỉ đổi **N**, mà đổi **cách phân bố hạt**.

Ví dụ:

### Uncertainty thấp

```text
          ●●●●●
        ●●●●●●●
         ●●●●
```

### Uncertainty cao

```text
   ●    ●       ●
      ●    ●
 ●          ●      ●
       ●
```

Tức là:

$$
N=f(Battery,\ Uncertainty,\ Terrain,\ SensorHealth)
$$

và:

$$
\Sigma_{proposal}=f(Uncertainty)
$$

---

# 12. Thêm Multi-Hypothesis Mode

Đây là nâng cấp rất mạnh nếu bạn thực sự muốn hệ thống có tính “survival”.

Trong tình huống:

```text
GPS unavailable
mag invalid
terrain ambiguous
```

không nên ép hệ thống chọn một nghiệm.

Cho phép:

```text
Hypothesis A
Hypothesis B
Hypothesis C
```

Ví dụ:

```text
           Particle Cloud
                 │
        ┌────────┼────────┐
        ▼        ▼        ▼
       H1       H2       H3
      0.51     0.31     0.18
```

Sau đó khi có sensor mới:

```text
measurement
   ↓
likelihood update
   ↓
H1 0.82
H2 0.12
H3 0.06
```

và merge/prune.

Đây là cách tự nhiên để xử lý địa hình đối xứng hoặc ambiguity.

---

# 13. Thêm "No False Confidence"

Đây sẽ là invariant số 7 tôi đề nghị thêm:

> **Confidence không được cao hơn chất lượng bằng chứng.**

Ví dụ:

```text
GPS LOST
MAG REJECTED
DEM INVALID
PDR DRIFT HIGH
```

nhưng:

```text
GNS = 0.94
```

→ rõ ràng sai.

Có thể định nghĩa:

$$
GNS
=
\min(
C_{position},
C_{attitude},
C_{velocity},
C_{sensor},
C_{model}
)
$$

hoặc một bound bảo thủ tương tự.

---

# 14. Thêm Watchdog cho Estimator

Hệ thống sinh tồn không nên giả định thread estimator luôn chạy.

Thêm:

```text
Estimator watchdog
        │
        ├── heartbeat
        ├── cycle time
        ├── state timestamp
        └── queue age
```

Nếu:

```text
last_update > threshold
```

→ `ESTIMATOR_STALE`.

Sau đó:

```text
NORMAL
 ↓
STALE
 ↓
DEGRADED
 ↓
EMERGENCY
```

Đây là lớp runtime safety còn thiếu.

---

# 15. Timestamp nên trở thành first-class data

Bạn đã có test timestamp discontinuity. 

Tôi khuyên mọi measurement đều có:

```cpp
struct MeasurementMeta {
    uint64_t timestamp_ns;
    uint32_t sequence;
    SensorId source;
};
```

và pipeline kiểm tra:

$$
t_{i+1}>t_i
$$

Không được để module tự lấy:

```cpp
std::chrono::now()
```

ở giữa pipeline.

**Sensor timestamp là source of truth.**

---

# 16. Thêm Deterministic Replay

Đây là một trong những nâng cấp tôi đánh giá cao nhất.

Tạo:

```text
record/
    session_001.slog
```

chứa:

```text
timestamp
sensor values
GPS
DEM queries
random seed
configuration
```

Sau đó:

```text
raw log
   ↓
replay
   ↓
same input
   ↓
same output
```

Cần target:

$$
State_{run1}=State_{run2}
$$

với cùng seed/config.

Điều này cực kỳ hữu ích để debug những lỗi PF/ESKF “lúc có lúc không”.

---

# 17. Deterministic Randomness cho Particle Filter

Đã muốn zero-allocation và deterministic performance thì nên tiến thêm:

```cpp
RandomEngine rng(seed);
```

không dùng random global.

Rescue incident có thể lưu:

```text
RNG seed = 0x...
```

Sau đó tái tạo chính xác trajectory của particle filter.

Cực kỳ hữu ích khi điều tra lỗi.

---

# 18. Thêm "Blackbox Incident Recorder"

Bạn đang có `local_blackbox`. Tôi sẽ nâng nó thành một **flight recorder** kiểu navigation system:

```text
INCIDENT
├── timestamp
├── navigation mode
├── state
├── covariance
├── sensor health
├── GPS status
├── PF statistics
├── estimator latency
├── faults
└── RNG seed
```

Khi người dùng sống sót thoát khỏi tình huống:

> có thể replay lại chính xác chuyện gì đã xảy ra.

Đây là feature rất giá trị cho debugging và validation.

---

# 19. Rescue Egress nên có "Staleness Flag"

Đừng chỉ gửi:

```text
LAT
LON
ALT
CE95
MODE
```

Thêm:

```text
FIX_AGE
POSITION_TIMESTAMP
POSITION_SOURCE
```

Ví dụ:

```text
POS: 10.xxxxxx,106.xxxxxx
AGE: 87s
SRC: PDR+TRN
CE95: 42m
MODE: DEGRADED
```

Cứu hộ sẽ biết:

> tọa độ này là hiện tại hay tọa độ cuối cùng đã cũ 2 phút.

Cực kỳ quan trọng.

---

# 20. Thêm "Position Provenance"

Tôi đề xuất canonical state có:

```cpp
enum class PositionSource {
    kGps,
    kPdr,
    kTrn,
    kGpsPdr,
    kGpsTrn,
    kPdrTrn,
    kLastKnownGood
};
```

Ví dụ:

```text
Position:
10.xxx
106.xxx

Source:
PDR + TRN

Age:
23.4 s

CE95:
31.2 m
```

Không chỉ `NavigationMode`.

---

# 21. Thêm Configuration Integrity

Một hệ thống navigation có thể sai không phải vì code mà vì config.

Ví dụ:

```text
mag_declination = wrong
step_length = wrong
DEM resolution = wrong
noise covariance = wrong
frame config = wrong
```

Vì vậy lưu:

$$
SHA256(configuration)
$$

vào blackbox/rescue metadata.

```text
CONFIG_HASH:
9af82....
```

Khi debug:

> “Thiết bị này đang chạy đúng profile nào?”

có câu trả lời ngay.

---

# 22. Tôi sẽ nâng cấp safety invariant từ 6 → 12

Bản hiện tại có 6 invariant. 

Tôi đề xuất:

```text
Invariant 1  No Teleportation
Invariant 2  No Null-Island Egress
Invariant 3  Core Platform Isolation
Invariant 4  Gated Sensor Ingestion
Invariant 5  Monotonic Uncertainty
Invariant 6  Degraded Mode Fail-Safe

Invariant 7  No False Confidence
Invariant 8  Covariance Validity
Invariant 9  Timestamp Monotonicity
Invariant 10 No NaN/Inf State
Invariant 11 Estimator Freshness
Invariant 12 Rescue Provenance Integrity
```

### Trong đó tôi đặc biệt coi:

**Invariant 7, 8, 9, 10, 11**

là bắt buộc.

---

# 23. Kiến trúc cuối cùng nên thành

```text
                     SENSOR INPUTS
                          │
                          ▼
               ┌────────────────────┐
               │ Sensor Validation  │
               │ Timestamp / Health │
               └──────────┬─────────┘
                          ▼
               ┌────────────────────┐
               │ ESKF / PDR / TRN   │
               │ Particle Filter    │
               └──────────┬─────────┘
                          ▼
               ┌────────────────────┐
               │ Canonical State    │
               │ + Covariance       │
               │ + Provenance       │
               └──────────┬─────────┘
                          ▼
               ┌────────────────────┐
               │ Integrity Monitor   │
               └──────────┬─────────┘
                          ▼
               ┌────────────────────┐
               │ Safety State Machine│
               └───────┬───────┬────┘
                       │       │
                 ┌─────▼──┐ ┌──▼─────────┐
                 │Renderer│ │Rescue Egress│
                 └────────┘ └─────────────┘
```

---

# 24. Và tôi sẽ thêm một lớp nữa: "Mission State"

Hiện có:

```text
GPS_AIDED
PDR_AIDED
TRN_AIDED
DEGRADED
LOST
EMERGENCY
```

rất tốt. 

Nhưng `NavigationMode` và `MissionState` nên tách.

Ví dụ:

```text
NavigationMode:
TRN_AIDED
```

nhưng:

```text
MissionState:
NORMAL
```

hoặc:

```text
NavigationMode:
TRN_AIDED

MissionState:
RESCUE_REQUIRED
```

Tức là **khả năng định vị** và **tình trạng nhiệm vụ** không phải một thứ.

---

# 25. Nếu chọn 10 nâng cấp đáng làm nhất

Tôi sẽ ưu tiên đúng thứ tự:

| Ưu tiên | Nâng cấp                         |
| ------: | -------------------------------- |
|      P0 | Integrity Monitor                |
|      P0 | Covariance Health Monitor        |
|      P0 | No NaN/Inf invariant             |
|      P0 | Timestamp Integrity              |
|      P0 | Sensor Health Model              |
|      P1 | ZUPT + ZARU                      |
|      P1 | GPS Temporal Consistency         |
|      P1 | Particle Degeneracy Recovery     |
|      P1 | Deterministic Replay             |
|      P1 | Rescue Provenance + Position Age |

Sau đó mới đến:

**Multi-Hypothesis PF → Adaptive Distribution → DEM Integrity → Watchdog → Configuration Hash → Incident Recorder.**

---

## Nhận định cuối

Bản v2.2 hiện tại đã có nền tảng khá tốt: **P0 correctness → P1 state continuity → P2 fault injection → P3 deterministic performance → P4 egress/renderer** là thứ tự hợp lý. 

Nhưng bước tiếp theo tôi sẽ không tiếp tục “thêm thuật toán định vị”. Tôi sẽ thêm **các thuật toán để phát hiện khi chính estimator đang sai**.

Đó là khác biệt lớn giữa:

> **Navigation Engine**

và

> **Safety-Critical Navigation Engine**.

Đặc biệt, nếu làm đến nơi đến chốn, tôi sẽ biến `Canonical Navigation State` thành một object kiểu:

```cpp
struct CanonicalNavigationState {
    Pose3D pose;
    Velocity3D velocity;

    Covariance covariance;

    NavigationMode mode;
    PositionSource position_source;

    SensorHealth sensor_health;
    IntegrityStatus integrity;

    double position_age_s;
    double timestamp_s;

    uint64_t configuration_hash;
    uint64_t rng_seed;
};
```

Khi đó **ESKF, PDR, PF, DEM, GUI, blackbox và rescue packet đều nói cùng một “ngôn ngữ trạng thái”**. Đây theo tôi là nâng cấp kiến trúc có giá trị nhất còn lại của OFF-GRID SURVIVAL.
