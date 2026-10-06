Mình đã đọc toàn bộ kế hoạch v2.2.0 → v2.3.0. Nhìn tổng thể, đây là một kế hoạch **đúng hướng và khá gần mức production**, nhưng có vài điểm trong chính thiết kế hiện tại cần sửa trước khi coi là “đã giải quyết triệt để”.

## Nhận định tổng thể

Mình đánh giá khoảng **8.7/10 về kiến trúc**, nhưng chỉ khoảng **7.5–8/10 về độ chắc chắn triển khai**.

Điểm mạnh nhất là kế hoạch đã chuyển từ kiểu “vá từng bug” sang sửa **invariant và runtime semantics**:

* Input sensor được sanitize ngay tại biên hệ thống. 
* Kinematics được nối vào activity classification thay vì để classifier tự suy đoán thiếu dữ liệu. 
* Particle Filter được biến thành **continuous propagation system**, thay vì chỉ update khi đi bộ. 
* C-API được xem như một runtime boundary thực sự, có watchdog, governor và safety wiring. 
* Có regression test tương ứng với từng bug, thay vì chỉ “build pass”. 

Đó là hướng rất đúng.

Nhưng mình **không khuyên triển khai nguyên xi bản này**.

---

# 1. Vấn đề lớn nhất: “100% lock-free / không UB” đang được khẳng định quá mạnh

Phần Triple Buffer nói:

> “Không bao giờ đọc ghi đồng thời trên cùng một buffer ⇒ Không bao giờ bị race condition hay torn-read.” 

Ý tưởng đúng, nhưng implementation đề xuất mới chỉ là **skeleton**, chưa đủ để kết luận lock-free và memory-safe tuyệt đối.

Đặc biệt:

```cpp
std::array<T, 3> buffers_;
std::atomic<uint8_t> read_idx_;
std::atomic<uint8_t> write_idx_;
std::atomic<uint8_t> clean_idx_;
```

vấn đề không nằm ở việc có 3 buffer hay không, mà nằm ở:

**protocol ownership + memory ordering + state transition.**

Ví dụ Writer:

```text
write → publish → swap
```

Reader:

```text
observe → acquire → consume
```

Nếu thứ tự atomic hoặc memory order không chuẩn, việc “không cùng buffer” trên giấy vẫn chưa đủ để chứng minh correctness.

### Định hướng

Đừng test:

> “chạy vài triệu vòng không crash”

mà phải có **formal ownership invariant**:

```text
READ   = buffer exclusively owned by Reader
WRITE  = buffer exclusively owned by Writer
CLEAN  = published but not currently owned
```

và invariant:

```text
READ != WRITE
WRITE != CLEAN
CLEAN != READ
```

Đồng thời test:

* ThreadSanitizer
* stress 1 writer / 1 reader
* random scheduling
* reader chậm
* writer nhanh gấp 10–100 lần reader
* wrap-around hàng triệu vòng

Và quan trọng: **đừng dùng cụm “100% lock-free” nếu chưa xác minh `std::atomic<uint8_t>::is_always_lock_free` trên tất cả target.**

---

# 2. Continuous Vehicle Particle Propagation là nâng cấp rất đáng giá — nhưng `noise = 0.20 * dt` chưa phải mô hình động học tốt

Đây là phần mình đánh giá cao nhất của v2.3.

Trước:

```text
PDR → di chuyển
PF  → đứng yên
```

Sau:

```text
PDR ─────┐
         ├─> particle propagation
DEM ─────┘
```

Đây mới thật sự là probabilistic navigation.

Kế hoạch hiện tại:

```cpp
disp_e = velocity_e * dt;
disp_n = velocity_n * dt;
disp_alt = vertical_speed * dt;
motion_noise = 0.20 * dt;
```



Tuy nhiên:

### `σ ∝ dt` chưa thật sự đúng về stochastic model.

Với quá trình random walk/Brownian-like:

```text
σ ∝ sqrt(dt)
```

Trong khi với uncertainty xuất phát từ acceleration noise / integrated kinematics thì variance thường phụ thuộc bậc cao hơn theo `dt`.

Vì vậy:

```cpp
noise_std = 0.20 * dt;
```

nên chuyển thành mô hình dựa trên **motion covariance**:

```text
Q_vehicle(dt, speed, acceleration, activity)
```

Ví dụ conceptually:

```text
Q = f(activity,
      speed,
      acceleration,
      turn_rate,
      GPS availability,
      IMU quality,
      dt)
```

Thay vì:

```text
mọi phương tiện = cùng một noise
```

---

# 3. MCMC Roughening đang bị gọi hơi “quá tên”

Phần:

```text
δE, δN ~ N(0, σ²)
```

với:

```text
σ = 0.5 m
```



thực chất giống **particle roughening / jittering** hơn là MCMC theo nghĩa đầy đủ.

Điều đó không xấu.

Ngược lại, nó có thể hoàn toàn đủ cho mục tiêu của hệ thống.

Nhưng mình sẽ đổi terminology:

```text
Adaptive Particle Roughening
```

hoặc:

```text
Post-Resampling Kernel Roughening
```

thay vì “MCMC Roughening” nếu không thực sự thực hiện Markov Chain Monte Carlo transition/kernel acceptance.

Điều này quan trọng nếu sau này bạn đưa dự án ra review nghiên cứu.

---

# 4. `σ = 0.5 m` cố định là điểm mình muốn sửa ngay

Terrain:

```text
flat → roughening
```

là đúng.

Nhưng:

```text
σ = 0.5 m
```

cho mọi tình huống thì hơi cơ học.

Ví dụ:

```text
GPS tốt       → không cần jitter lớn
DEM rõ        → jitter nhỏ
GPS mất       → jitter lớn
particle ESS thấp → jitter tăng
vehicle speed cao → uncertainty tăng
```

Mình khuyên chuyển thành:

$$
\sigma_{rough}
=
f(ESS,\;terrain,\;speed,\;sensor\_quality,\;dt)
$$

Trong đó đặc biệt nên lấy **ESS (Effective Sample Size)** làm tín hiệu chính.

Ví dụ concept:

```text
ESS rất thấp
→ particle degeneracy
→ resample
→ roughening tăng

ESS ổn định
→ roughening giảm
```

Như vậy hệ thống trở thành **adaptive**, không phải hard-coded.

---

# 5. Activity classifier hiện tại đang có nguy cơ “rule explosion”

Đoạn:

```cpp
if (!frame.has_baro) {
    if (pitch_rad < 0.14 ||
        stair_feat.vertical_jerk_m_s3 < 40.0F) {
        scores[3] *= 0.10F;
    }
}
```



giải quyết Bug 3 rất trực tiếp.

Nhưng khi số activity tăng lên 11 loại, kiểu:

```text
if A
if B
if C
if !baro
if !gps
if speed > X
...
```

sẽ nhanh chóng biến classifier thành một **decision-tree khổng lồ khó bảo trì**.

Mình khuyên tiến hóa nó thành:

```text
Feature extraction
        ↓
Feature confidence
        ↓
Activity likelihood
        ↓
Temporal smoothing
        ↓
FSM / mode transition
```

Tức là thay vì:

```cpp
scores[3] *= 0.10;
```

hãy coi mỗi sensor là một nguồn evidence với confidence.

Ví dụ:

```text
Baro unavailable
→ baro confidence = 0

IMU available
→ pitch confidence = high

GPS available
→ horizontal velocity confidence = high
```

Sau đó classifier hợp nhất evidence.

Đây sẽ scale tốt hơn rất nhiều.

---

# 6. Bug 5: `60 m/s` xử lý được false-positive, nhưng chưa xử lý physics consistency

Kế hoạch nâng:

```text
walking/running: 15 m/s
vehicle: 60 m/s
```



Đúng với mục tiêu bug.

Nhưng bản chất tốt hơn là:

```text
velocity_limit = f(activity, acceleration, turn_rate, confidence)
```

Ví dụ xe không thể:

```text
0 → 60 m/s
```

trong một timestep mà không có acceleration tương ứng.

Do đó nên bổ sung:

```text
velocity plausibility
+
acceleration plausibility
+
jerk plausibility
```

Thành:

```text
Kinematic Integrity Layer
```

Cách này sẽ mạnh hơn rất nhiều so với một `max_velocity_limit`.

---

# 7. Watchdog 1.5s là đúng về safety, nhưng nên tách “stale” và “dead”

Hiện tại:

```text
>1.5s → estimator stale
```



Mình khuyên dùng 3 trạng thái:

```text
HEALTHY
DEGRADED
STALE
DEAD
```

Ví dụ:

```text
0–500 ms    HEALTHY
500–1500 ms DEGRADED
>1500 ms    STALE
>5 s        DEAD
```

Vì:

> “sensor chậm”

khác hoàn toàn:

> “sensor chết”.

Đây sẽ rất hữu ích cho Rescue/SOS logic.

---

# 8. Resource Governor nên được nâng từ “pin < 5%” thành Adaptive Compute Governor

Hiện tại:

```text
battery < 5%
→ giảm particle count
```



Đây là bước đầu.

Nhưng hệ thống của bạn có một thứ rất quý:

**PF có thể thay đổi precision/particle budget.**

Mình sẽ thiết kế:

```text
Resource Governor
       │
       ├── Battery
       ├── CPU load
       ├── thermal state
       ├── sensor quality
       ├── ESS
       └── activity
              ↓
       particle budget
              ↓
       update frequency
              ↓
       roughening quality
```

Ví dụ:

```text
Battery 80%
CPU 30%
→ 10,000 particles

Battery 15%
CPU 70%
→ 5,000 particles

Battery 5%
→ 1,500 particles
```

Nhưng khi terrain phức tạp hoặc uncertainty tăng:

```text
ESS collapse
→ governor cho phép tăng particle budget tạm thời
```

Đây mới đúng nghĩa **adaptive resource engine**.

---

# 9. Thứ tự 5 phase hiện tại nên chỉnh nhẹ

Hiện tại:

```text
GĐ1 Integrity
GĐ2 Kinematics
GĐ3 Particle
GĐ4 C-API
GĐ5 Verification
```



Mình sẽ đổi thành:

```text
PHASE 0 — Baseline & Instrumentation
        ↓
PHASE 1 — Integrity/Sanitization
        ↓
PHASE 2 — Kinematics + Activity
        ↓
PHASE 3 — PF Continuous Propagation
        ↓
PHASE 4 — Runtime/C-API/Resource Governance
        ↓
PHASE 5 — Adversarial Validation
        ↓
PHASE 6 — Performance Qualification
```

**Phase 0** cực kỳ quan trọng.

Trước khi sửa hãy ghi lại baseline:

```text
PF latency
PDR latency
CPU
memory
particle count
ESS
position RMSE
heading error
activity confusion matrix
sensor dropout recovery
```

Nếu không có baseline thì sau này rất khó chứng minh:

> v2.3 tốt hơn v2.2 bao nhiêu.

---

# 10. Test hiện tại vẫn còn thiên về “bug regression”, chưa đủ “system validation”

Các test:

```cpp
EXPECT_GT(...)
EXPECT_FALSE(...)
EXPECT_EQ(...)
EXPECT_TRUE(...)
```



rất tốt để đóng 12 bug.

Nhưng navigation engine cần thêm **property-based / scenario-based testing**.

Đặc biệt mình sẽ tạo một:

## `Scenario Simulation Harness`

Có các scenario:

```text
S01 Flat walking
S02 Stair climbing
S03 Elevator
S04 Escalator
S05 Bicycle
S06 Car 20 km/h
S07 Car 80 km/h
S08 Car 120 km/h
S09 GPS dropout
S10 IMU NaN
S11 Barometer dropout
S12 Sensor freeze
S13 Long vehicle journey
S14 Circular wandering
S15 Repeated resampling
S16 Degenerate particle cloud
```

Mỗi scenario xuất:

```text
position error
velocity error
heading error
ESS
particle spread
activity accuracy
integrity state
CPU
latency
battery budget
```

Đây sẽ biến dự án từ:

> “một C++ navigation library có nhiều test”

thành:

> **một navigation system có verification framework.**

---

# Định hướng kiến trúc mình khuyên dùng

Mình sẽ đưa hệ thống tiến tới mô hình này:

```text
                 ┌─────────────────────┐
                 │   Sensor Integrity  │
                 │ NaN / Inf / Stale   │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Feature Extraction  │
                 │ IMU / Baro / GPS    │
                 │ Velocity / Terrain  │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Activity Inference  │
                 │ + Confidence        │
                 └──────────┬──────────┘
                            │
               ┌────────────┴────────────┐
               ▼                         ▼
       ┌───────────────┐        ┌────────────────┐
       │ PDR / Kinematic│        │ Terrain / DEM  │
       │ Propagation    │        │ Constraint     │
       └───────┬───────┘        └────────┬───────┘
               └────────────┬─────────────┘
                            ▼
                 ┌─────────────────────┐
                 │ Particle Filter     │
                 │ Propagation         │
                 │ Resampling          │
                 │ Adaptive Roughening │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Integrity / Safety  │
                 │ Confidence / Watchdog│
                 └──────────┬──────────┘
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
       Rescue State                 Resource Governor
              │                           │
              └─────────────┬─────────────┘
                            ▼
                       C API / UI
```

Đây là kiến trúc mà mình nghĩ **nên trở thành v2.4/v2.5**, thay vì chỉ dừng ở việc đóng 12 bug của v2.3.

---

# Một thay đổi rất quan trọng nữa

Trong tài liệu đang có câu:

> “Điều này giúp `extractUncertaintyState()` phát ra tâm cụm di chuyển bám sát theo xe 100%.” 

Mình khuyên **bỏ chữ `100%`**.

Navigation probabilistic không nên cam kết:

```text
100% bám sát
```

mà nên cam kết bằng metric:

```text
RMSE
95% CEP
95% confidence ellipse coverage
NEES
NIS
```

Đây là khác biệt rất lớn giữa một hệ thống “trông có vẻ đúng” và một hệ thống navigation có thể được đánh giá khoa học.

---

# Lộ trình mình đề xuất

### v2.2.1

**Safety correctness**

Đóng toàn bộ:

```text
NaN/Inf
log-weight
blackbox overflow
Null Island
benchmark timing
```

### v2.2.2

**Kinematic correctness**

Đóng:

```text
activity
velocity
baro dropout
vehicle speed
freshness
```

### v2.3.0

**Probabilistic motion**

Triển khai:

```text
continuous propagation
adaptive Q
ESS
resampling
roughening
```

### v2.3.1

**Runtime correctness**

Triển khai:

```text
triple buffer
watchdog
C API
resource governor
```

### v2.4

**Adaptive Navigation Engine**

Đây mới là bước mình nghĩ rất đáng đầu tư:

```text
Adaptive Q
Adaptive particle count
Adaptive roughening
sensor confidence fusion
dynamic activity model
multi-level degradation
```

### v2.5

**Verification-grade Navigation**

Thêm:

```text
scenario simulator
Monte Carlo testing
RMSE / CEP / NEES / NIS
fault injection
stress testing
long-duration endurance
performance qualification
```

---

## Kết luận

Cái hay nhất của kế hoạch này là bạn đã bắt đầu đi đúng hướng **“engine hóa”** hệ thống.

v2.2 có vẻ vẫn là:

> nhiều module navigation nối với nhau.

Còn v2.3 bắt đầu trở thành:

> **một probabilistic state-estimation engine có integrity, runtime governance và fault handling.**

Đó là một bước nhảy kiến trúc khá lớn.

Tuy nhiên, mình sẽ ưu tiên sửa **4 thứ trước khi code**:

**(1)** formalize Triple Buffer ownership/memory ordering,
**(2)** thay `0.20*dt` bằng motion covariance `Q(dt, activity, speed)`,
**(3)** biến roughening thành adaptive theo ESS + sensor uncertainty,
**(4)** xây Scenario/Monte-Carlo Harness trước khi tuyên bố v2.3 “production-ready”.

Nếu làm được 4 điểm này, mình sẽ nâng đánh giá kiến trúc từ khoảng **8.7/10 lên khoảng 9.3–9.5/10**, vì lúc đó hệ thống không chỉ “sửa bug” mà đã có nền tảng của một **adaptive probabilistic navigation platform**.
