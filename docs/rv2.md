Tôi đã đọc bản `implementation_plan.md`. Bản này đã tiến thêm một bước khá rõ: nó không chỉ nhận diện **đi bộ/chạy/cầu thang/phương tiện**, mà còn nối trực tiếp activity classifier vào `InstantaneousVelocityEstimator` 100 Hz và `CanonicalNavigationState`. 

Tuy nhiên, tôi thấy phần **cầu thang và phương tiện** vẫn còn vài giả định hơi “thẳng tay”. Nếu sửa chúng, hệ thống sẽ đáng tin hơn rất nhiều trong môi trường thật.

# 1. Nhận xét quan trọng nhất: classifier hiện tại đang dùng quá ít bằng chứng

Hiện bản v3.1 dùng:

* cadence
* `T_rise`
* P2P / free-fall
* pitch
* năng lượng >12 Hz
* barometer nếu có. 

Ý tưởng đúng, nhưng tôi **không khuyên dùng rule cứng kiểu**:

```text
Pitch -40°
+
T_rise 240 ms
=
STAIRS_UP
```

hoặc:

```text
25 Hz vibration
+
cadence 0
=
VEHICLE
```

Bởi vì thực tế:

```text
stairs + phone in pocket
stairs + phone in hand
stairs + phone in backpack
stairs + người quay người
```

sẽ tạo IMU hoàn toàn khác nhau.

Tương tự:

```text
xe máy chạy đường xấu
ô tô chạy đường xấu
đi bộ trên cầu thép
chạy
thang cuốn
```

đều có thể tạo năng lượng cao ở dải tần mà bạn đang dùng.

### Vì vậy nên chuyển từ:

> **Single-feature threshold**

sang:

> **Multi-modal evidence fusion + temporal FSM**

---

# 2. Tôi đề xuất kiến trúc Motion Classifier mới

Thay vì:

```text
IMU → vài feature → activity
```

hãy dùng:

```text
                    SensorFrame
                         │
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
      IMU              Baro              GNSS
        │                │                │
        ▼                ▼                ▼
 Motion Features   Vertical Motion    Speed/Course
        │                │                │
        └────────────────┼────────────────┘
                         ▼
                 Feature Fusion
                         │
                         ▼
                Candidate Activities
                         │
                         ▼
               Temporal State Machine
                         │
                         ▼
             MotionActivity + Confidence
```

Quan trọng là classifier phải biết:

> **“Tôi đang quan sát một đoạn chuyển động”**

chứ không phải:

> “Một frame này giống cầu thang.”

---

# 3. Nâng cấp nhận diện cầu thang

Đây là phần tôi khuyên đầu tư mạnh nhất.

## 3.1. Đừng chỉ dùng Pitch

Bản hiện tại lấy **Pitch âm sâu -35° → -50°** làm feature lên cầu thang. 

Điều này khá nguy hiểm.

Với smartphone:

```text
người dùng cầm máy thẳng
→ pitch khác

người dùng bỏ túi
→ pitch khác

người dùng cầm máy ngang
→ pitch khác
```

Thậm chí cùng một cầu thang, pitch của thiết bị có thể không phản ánh góc dốc.

### Tôi đề xuất thêm:

$$
\theta_{gravity}
=
\angle(\text{device axis},g)
$$

nhưng classifier **không được phụ thuộc tuyệt đối vào orientation của thiết bị**.

Thay vào đó hãy biến IMU về **gravity-aligned frame**:

```text
Body IMU
   ↓
ESKF orientation
   ↓
Gravity compensation
   ↓
Vertical / Horizontal acceleration
```

Sau đó lấy:

$$
a_v = a\cdot \hat g
$$

và:

$$
a_h = a-a_v\hat g
$$

Lúc này người dùng xoay điện thoại thế nào ít ảnh hưởng hơn.

---

# 4. Cầu thang nên có "Step Geometry Signature"

Mỗi bước cầu thang có cấu trúc tương đối đặc trưng:

```text
push
 ↓
rise
 ↓
impact
 ↓
settle
 ↓
next step
```

Ta có thể trích:

$$
T_{step}
$$

$$
\Delta h_{step}
$$

$$
J_{step}
$$

$$
P2P_{step}
$$

$$
E_{step}
$$

và quan trọng nhất:

$$
R_{vertical}
=
\frac{\Delta h_{vertical}}
{\text{step count}}
$$

Nếu đang leo:

$$
\Delta h > 0
$$

và nếu đang xuống:

$$
\Delta h < 0
$$

### Cực kỳ quan trọng:

Không nên hard-code:

$$
H_{riser}=0.16m
$$

cho mọi cầu thang.

Bản hiện tại đang dùng khoảng 0.16–0.17 m cho riser.  

Nên dùng:

$$
H_{riser}^{est}
=
\frac{\Delta h_{baro}}
{N_{steps}}
$$

rồi lọc:

$$
H_{riser}^{est}
\rightarrow
H_{riser}^{personal/context}
$$

Ví dụ:

```text
barometer:
+1.74 m

detected:
11 steps

estimated:
0.158 m/step
```

Nếu sau 3–5 bậc kết quả ổn định, classifier có thể “khóa” riser estimate cho đoạn cầu thang đó.

---

# 5. Thêm phát hiện "Landing"

Đây là một nâng cấp rất hay mà kế hoạch hiện tại chưa có.

Cầu thang thực:

```text
stairs
stairs
stairs
stairs
──── landing ────
stairs
stairs
stairs
```

Trong một đoạn landing:

```text
vertical velocity ≈ 0
step cadence ≈ 0
```

Nhưng đây **không phải Stationary**.

Cần state:

```text
kStairsAscending
        ↓
kStairLanding
        ↓
kStairsAscending
```

Nếu không, classifier có thể:

```text
STAIRS_UP
↓
STATIONARY
↓
STAIRS_UP
```

và làm velocity/state continuity bị giật.

---

# 6. Thêm `kEscalator`

Tôi thực sự khuyên thêm.

Hiện enum có:

```text
Stationary
Walking
Running
Stairs Up
Stairs Down
Vehicle
Elevator/Cable
```



Nhưng **thang cuốn** rất dễ bị nhầm với:

```text
stairs ascent
```

Cách phân biệt:

### Cầu thang bộ

$$
step\ cadence > 0
$$

$$
vertical\ motion \approx periodic
$$

### Đứng trên thang cuốn đi lên

$$
step\ cadence \approx 0
$$

nhưng:

$$
v_z > 0
$$

và barometer tăng đều.

Do đó:

```text
v_z ≠ 0
+
cadence ≈ 0
+
baro trend ≠ 0
=
ESCALATOR / ELEVATOR candidate
```

---

# 7. Cầu thang nên có Step Counter riêng

Hiện `stairs_step_count` đang nằm trong `MotionActivityEstimate`. 

Tôi khuyên nâng cấp thành:

```cpp
struct StairState {
    uint32_t step_count;
    uint32_t landing_count;

    float estimated_riser_m;
    float cumulative_vertical_m;

    float cadence_hz;
    float confidence;
};
```

Và lưu:

```text
last_step_timestamp
step_interval
step_direction
```

Như vậy navigation có thể sử dụng:

$$
\Delta z
=
N_{step}\times H_{riser}
$$

thay vì đo vertical velocity một cách mù.

---

# 8. Nhận diện phương tiện: 12 Hz chưa đủ

Đây là điểm tôi muốn sửa khá mạnh.

Bản hiện tại dùng:

> năng lượng rung động động cơ >12 Hz để nhận diện xe. 

Tôi sẽ **không dùng `E_motor > threshold` như detector chính**.

Bởi vì:

```text
xe máy
ô tô
xe địa hình
tàu
máy rung
đi bộ trên nền rung
```

có spectral signature khác nhau.

Nên tính:

$$
E_{low}
$$

$$
E_{mid}
$$

$$
E_{high}
$$

và ratio:

$$
R_{HF}
=
\frac{E_{12-40Hz}}
{E_{0.5-12Hz}}
$$

Thêm spectral entropy:

$$
H_{spec}
=
-\sum p_i\log p_i
$$

và dominant frequency:

$$
f_{peak}
$$

---

# 9. Nhưng vehicle detector phải nhìn cả chuyển động vĩ mô

Đây mới là thứ quan trọng.

Một xe đang chạy thường có:

```text
low-frequency body motion
+
high-frequency vibration
+
speed > 0
+
no human cadence
```

Do đó:

$$
VehicleScore =
f(
Spectral,
Speed,
Cadence,
Jerk,
Barometer,
Orientation
)
$$

Ví dụ:

```text
cadence ≈ 0
+
speed = 8 m/s
+
high-frequency vibration
+
repeated road shocks
→ VEHICLE
```

Khác với:

```text
cadence ≈ 0
+
speed ≈ 0
+
baro stable
+
high vibration
→ phone/device vibration
```

---

# 10. Thêm GNSS Doppler / Speed khi có GPS

Nếu GPS có velocity:

$$
v_{GNSS}
$$

thì đây là tín hiệu cực kỳ mạnh để phân biệt:

```text
walking
running
vehicle
```

Ví dụ:

```text
v < 2 m/s
→ likely walk

v = 2–5 m/s
→ walk/run/cycle ambiguity

v > 6 m/s
→ vehicle candidate
```

Nhưng **đừng hard-code các ngưỡng này thành chân lý**; nên dùng làm evidence.

Đặc biệt GPS course + IMU heading còn giúp phát hiện:

```text
vehicle movement direction
```

---

# 11. Tôi sẽ thêm `kCycling`

Vì hiện tại:

```text
walking
running
stairs
vehicle
```

có một vùng mù là **xe đạp**.

Xe đạp có:

```text
speed > walking
cadence periodic
body vibration
```

rất dễ khiến classifier nhầm:

```text
RUNNING
```

hoặc:

```text
VEHICLE
```

Thêm:

```text
kCycling
```

sẽ hợp lý hơn cho survival navigation.

---

# 12. Đừng phân biệt "xe máy" và "ô tô" quá sớm

Tôi sẽ giữ tầng 1:

```text
kVehicleWheeled
```

rồi tầng 2:

```text
VehicleSubtype:
    Unknown
    Motorcycle
    Car
    Bus
    Train
    Bicycle
```

Chỉ phân loại subtype khi confidence đủ.

Ví dụ:

```text
VehicleWheeled 0.94
Motorcycle      0.53
Car             0.31
```

Canonical state chỉ cần:

```text
VEHICLE_WHEELED
```

trừ khi có đủ evidence để nói sâu hơn.

---

# 13. Elevator và cable car cũng không nên gộp quá sớm

Hiện enum dùng:

```text
kElevatorCable
```



Nhưng hai loại này có dynamics khác nhau.

### Elevator:

$$
v_z(t)
$$

có profile:

```text
0
→ acceleration
→ constant vertical speed
→ deceleration
→ 0
```

### Cable car:

có thể:

```text
horizontal movement
+
vertical component
+
periodic tower-induced vibration
```

Do đó nên:

```text
kElevator
kCableTransport
```

hoặc ít nhất:

```text
VerticalTransport
```

ở lớp activity và subtype ở lớp sau.

---

# 14. Tạo "Motion Transition Graph"

Đây là nâng cấp tôi đánh giá cực cao.

Các trạng thái không thể chuyển ngẫu nhiên.

Ví dụ:

```text
STATIONARY
   │
   ├── WALKING
   ├── RUNNING
   └── VEHICLE

WALKING
   │
   ├── STAIRS_UP
   ├── STAIRS_DOWN
   ├── RUNNING
   └── VEHICLE

STAIRS_UP
   │
   ├── STAIR_LANDING
   └── WALKING

VEHICLE
   │
   └── STATIONARY
```

Không cho:

```text
STAIRS_UP
   ↓ 1 frame
VEHICLE
```

trừ khi bằng chứng rất mạnh.

Đây là **temporal hysteresis**, chống classifier "nhảy loạn".

---

# 15. Nâng cấp Confidence thành 3 lớp

Đừng chỉ:

```text
activity_confidence
```

Nên có:

```cpp
struct MotionActivityEstimate {
    MotionActivity current_activity;

    float raw_confidence;
    float temporal_confidence;
    float fused_confidence;

    float transition_confidence;
};
```

Ví dụ:

```text
Raw:
STAIRS_UP = 0.82

Temporal:
STAIRS_UP = 0.94

Fused:
STAIRS_UP = 0.90
```

Chỉ transition khi:

$$
C_{fused}>T_{enter}
$$

và thoát state khi:

$$
C_{fused}<T_{exit}
$$

với:

$$
T_{enter}>T_{exit}
$$

→ hysteresis.

---

# 16. Velocity estimator hiện tại có một điểm tôi không thích

Bản hiện tại dùng:

$$
v_{horiz}
=
L_{stride}f_{cadence}
(1+0.3\sin(2\pi ft))
$$

cho walking/running. 

Tôi khuyên **không đưa sóng sin giả định 30% vào navigation state thật**.

Nó có thể đẹp trên demo nhưng không nhất thiết phản ánh vận tốc người thật.

Nên dùng:

$$
v =
\frac{L_{step}}{\Delta t_{step}}
$$

với:

$$
L_{step}=f(
height,\ cadence,\ activity,\ acceleration
)
$$

và sau đó lọc:

$$
v_{filtered}
=
OneEuro(v)
$$

hoặc bộ lọc tương tự có bounded response.

Sóng vi mô nên chỉ dùng cho **rendering/prediction**, không nên làm nguồn ground truth cho navigation.

---

# 17. Với phương tiện, tuyệt đối không chỉ dùng Leaky Integrator

Bản hiện tại đề xuất:

$$
a_{nav}
=
R_{nb}a_{body}-g
$$

sau đó leaky integrate với:

$$
\alpha=0.98
$$



Cái này phù hợp như một **fallback**, nhưng không đủ để tạo vận tốc xe đáng tin lâu dài.

Vì:

$$
v(t)=\int a(t)dt
$$

bias cực nhỏ cũng tích lũy thành sai số lớn.

### Tôi đề xuất thứ tự ưu tiên:

```text
GNSS Doppler velocity
        ↓
Wheel/vehicle evidence nếu có
        ↓
IMU strapdown
        ↓
Motion model
        ↓
Leaky velocity fallback
```

Tức là INS không phải nguồn duy nhất.

---

# 18. Thêm Vehicle Motion Model

Khi đã xác định:

```text
VehicleWheeled
```

thì không còn áp dụng human walking model.

Dùng:

$$
x_{t+1}=f_{vehicle}(x_t,u_t)
$$

với:

```text
speed
heading
acceleration
yaw-rate
```

Ví dụ mô hình tối giản:

$$
\dot x=v\cos\psi
$$

$$
\dot y=v\sin\psi
$$

$$
\dot\psi=\omega_z
$$

Đây gần với **kinematic bicycle/unicycle model** và hợp lý hơn việc chỉ tích phân acceleration.

---

# 19. Một nâng cấp cực hay: phát hiện "được mang trên phương tiện"

Có hai trường hợp:

```text
A. người tự đi xe
B. điện thoại được mang trên xe
```

navigation cần biết:

> “Tôi đang di chuyển bằng phương tiện”, không cần biết người dùng có đang thao tác hay không.

Do đó thêm:

```text
PassengerMotionModel
```

Nếu:

```text
speed high
cadence absent
vehicle vibration strong
```

→ vehicle.

Nếu:

```text
speed high
cadence periodic
```

→ cycling/running candidate.

---

# 20. Thêm phát hiện "đỗ xe / dừng xe"

Vehicle không phải lúc nào cũng:

```text
VEHICLE + speed > 0
```

Xe dừng đèn đỏ:

```text
vehicle
speed = 0
```

nhưng vẫn có rung động.

Do đó:

```text
VEHICLE_MOVING
VEHICLE_STOPPED
```

nên là **sub-state**, không nhất thiết là activity enum mới.

---

# 21. Một thứ rất đáng thêm: Context Window 2 tầng

Hiện dùng cửa sổ 64 mẫu ở 100 Hz:

$$
64/100=0.64s
$$



Tôi sẽ dùng:

```text
Fast window:
0.5–0.75 s
```

để nhận:

```text
step impact
jerk
vibration
```

và:

```text
Slow window:
3–5 s
```

để nhận:

```text
walking trend
stairs trend
vehicle trend
elevator profile
```

Sau đó:

```text
fast features
+
slow context
→ activity
```

Điều này sẽ giảm nhầm lẫn rất mạnh.

---

# 22. Tôi đề xuất sửa enum thành

```cpp
enum class MotionActivity : uint8_t {
    kStationary = 0,

    kWalkingFlat,
    kRunning,

    kStairsAscent,
    kStairsDescent,
    kStairLanding,

    kEscalator,

    kCycling,

    kVehicleWheeled,
    kElevator,
    kCableTransport,

    kUnknown
};
```

và subtype riêng:

```cpp
enum class VehicleSubtype : uint8_t {
    kUnknown,
    kBicycle,
    kMotorcycle,
    kCar,
    kBus,
    kTrain
};
```

Không cần ép engine phải xác định subtype ngay.

---

# 23. Thêm "Activity Evidence"

Tôi rất khuyên lưu lý do classifier đưa ra quyết định.

Ví dụ HUD:

```text
ACTIVITY: STAIRS_UP
CONF: 0.93

Evidence:
Cadence       1.74 Hz
Vertical Δh   +0.158 m/step
Baro trend    +1.62 m
Impact        0.82
Terrain       stair-like
```

Hoặc:

```text
ACTIVITY: VEHICLE
CONF: 0.91

Speed         12.4 m/s
Cadence       0.00 Hz
HF vibration  0.71
GPS velocity  11.9 m/s
Yaw rate      low
```

Khi classifier sai, bạn sẽ **biết nó sai vì feature nào**.

---

# 24. Verification cũng phải nâng cấp

Bộ test hiện tại khá tốt nhưng còn thiên về synthetic thresholds. 

Tôi đề xuất thêm:

### Stair tests

```text
Straight stairs
Steep stairs
Shallow stairs
Different riser heights
Landing
Half-turn stairs
Outdoor stairs
Phone pocket
Phone hand
Phone backpack
Ascending → landing → descending
```

### Vehicle tests

```text
Car smooth road
Car rough road
Motorcycle
Bus
Train
Vehicle stopped
Vehicle accelerating
Vehicle braking
Vehicle turning
Phone stationary inside vehicle
Walking beside road
Running on rough surface
```

### Confusion matrix

Không chỉ:

```text
PASS
```

mà:

$$
Precision,\ Recall,\ F1
$$

và:

$$
ConfusionMatrix
$$

Ví dụ:

| Actual / Pred | Walk | Run | Stairs Up | Stairs Down | Vehicle |
| ------------- | ---: | --: | --------: | ----------: | ------: |
| Walk          |  96% |  2% |        1% |          1% |      0% |
| Run           |   2% | 95% |        1% |          0% |      2% |
| Stairs Up     |   1% |  1% |       96% |          1% |      1% |
| Stairs Down   |   1% |  0% |        2% |         95% |      2% |
| Vehicle       |   0% |  1% |        0% |          0% |     99% |

---

# 25. Một KPI cực kỳ quan trọng mà tài liệu hiện tại chưa có

Không chỉ đo:

> nhận diện đúng hay sai.

Cần đo:

### Time-to-Classification

Ví dụ:

```text
stairs starts
      ↓
  120 ms
      ↓
STAIRS_UP confidence > 0.8
```

và:

### False Transition Rate

$$
FTR=
\frac{\text{false activity transitions}}
{\text{minute}}
$$

Đây mới là thứ ảnh hưởng trực tiếp tới navigation.

---

# 26. Kiến trúc tôi khuyên chốt

```text
                        SENSOR FRAME
                             │
          ┌──────────────────┼─────────────────┐
          ▼                  ▼                 ▼
         IMU                BARO              GNSS
          │                  │                 │
          ▼                  ▼                 ▼
   Gravity-aligned      Vertical trend    Speed/course
      features               │                 │
          │                  │                 │
          └──────────────────┼─────────────────┘
                             ▼
                    Feature Extraction
                             │
             ┌───────────────┼────────────────┐
             ▼               ▼                ▼
         Step model      Stair model      Vehicle model
             │               │                │
             └───────────────┼────────────────┘
                             ▼
                    Evidence Fusion
                             │
                             ▼
                  Temporal Activity FSM
                             │
                             ▼
              MotionActivity + Confidence
                             │
       ┌─────────────────────┼──────────────────────┐
       ▼                     ▼                      ▼
   PDR Model            Stair Model           Vehicle Model
       │                     │                      │
       └─────────────────────┼──────────────────────┘
                             ▼
                 Instantaneous Velocity
                             │
                             ▼
                   Canonical Nav State
```

---

# 27. Tôi sẽ bổ sung thêm 8 module

Theo cấu trúc project hiện tại, tôi đề xuất:

```text
core/
├── kinematics/
│   ├── motion_activity_classifier.*
│   ├── stair_motion_detector.*
│   ├── stair_step_tracker.*
│   ├── stair_geometry_estimator.*
│   ├── vehicle_motion_detector.*
│   ├── vehicle_motion_model.*
│   ├── vertical_transport_detector.*
│   └── motion_transition_fsm.*
```

và:

```text
types.hpp
    MotionActivity
    VehicleSubtype
    StairState
    VehicleState
    MotionEvidence
```

---

# 28. Một thay đổi tôi đặc biệt khuyên: `MotionEvidence`

```cpp
struct MotionEvidence {
    float cadence_hz;

    float vertical_energy;
    float horizontal_energy;

    float impact_score;
    float freefall_score;

    float hf_vibration_ratio;
    float spectral_entropy;

    float baro_vertical_rate;
    float gnss_speed;
    float gnss_course;

    float stair_score;
    float vehicle_score;
    float elevator_score;
};
```

Sau đó:

```cpp
MotionActivityEstimate
```

không còn là “black box”.

Nó có thể giải thích được:

```text
stairs_score = 0.91
vehicle_score = 0.08
elevator_score = 0.04
```

---

# 29. Có một nâng cấp còn quan trọng hơn tất cả: cá nhân hóa

Hai người:

```text
User A: 1.55m
User B: 1.90m
```

không có:

$$
L_{stride,A}=L_{stride,B}
$$

và:

$$
H_{step}
$$

cũng khác tùy môi trường.

Do đó:

```text
UserProfile
├── height
├── estimated stride
├── cadence baseline
├── walking signature
├── running signature
└── historical step length
```

rất đáng để xây.

Có thể tự học:

$$
L_{stride}
=
f(height,cadence,a_{rms})
$$

và cập nhật online.

---

# 30. Thứ tự nâng cấp tôi khuyên dùng

Không cần nhảy ngay vào machine learning.

### P0

```text
Gravity-aligned IMU
+
Temporal FSM
+
Activity hysteresis
```

### P1

```text
Stair step tracker
+
Barometric Δh
+
Adaptive riser estimator
+
Landing detection
```

### P1

```text
Vehicle spectral features
+
GNSS speed
+
Cadence suppression
+
Vehicle motion model
```

### P2

```text
Escalator
+
Elevator
+
Cable transport
+
Cycling
```

### P2

```text
Personalized stride
+
Adaptive activity thresholds
```

### P3

Sau khi có dataset đủ lớn mới cân nhắc:

```text
TinyML / small classifier
```

như:

```text
Gradient Boosting
Small Random Forest
Tiny MLP
```

nhưng **giữ rule/FSM làm safety layer**.

---

## Kết luận

Bản v3.1 hiện tại đã có một nền khá tốt: activity classifier, stair biomechanics, vehicle model và instantaneous 3D velocity đã được nối vào cùng pipeline. 

Nhưng tôi sẽ **không chốt implementation hiện tại** vì ba giả định này hơi nguy hiểm:

$$
Pitch \approx Stair
$$

$$
12Hz+Vibration \approx Vehicle
$$

$$
a_{vehicle}\rightarrow\int\rightarrow v
$$

Ba cái đều hữu ích như **evidence**, nhưng không nên là **ground truth**.

Tôi sẽ biến hệ thống thành:

> **Motion Evidence Fusion + Temporal FSM + Activity-specific Motion Models + Confidence/Uncertainty**

và thêm ít nhất:

**Stair Landing, Adaptive Riser Estimation, Escalator, Cycling, Vehicle Motion Model, GNSS velocity fusion, Vehicle Stop State, Multi-timescale windows và Motion Evidence logging.**

Khi đó OFF-GRID SURVIVAL sẽ có một pipeline rất mạnh:

```text
Đứng yên
   ↓
Đi bộ
   ↓
Cầu thang ↑
   ↓
Landing
   ↓
Cầu thang ↓
   ↓
Mặt phẳng
   ↓
Xe
   ↓
Xe dừng
   ↓
Xuống xe
   ↓
Đi bộ
```

và quan trọng nhất là **navigation không bị reset hoặc đổi mô hình đột ngột ở mỗi transition**. `InstantaneousVelocity` lúc này cũng có thể chuyển model một cách liên tục thay vì nhảy từ PDR sang INS theo kiểu on/off. Bản kế hoạch hiện đã đặt nền cho phần này ở 100 Hz. 
