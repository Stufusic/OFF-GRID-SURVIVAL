Mình xem kỹ bản này rồi. So với bản v2.3 trước, tài liệu này **đang nhắm đúng vào một lỗi rất khó: phân biệt Ascent/Descent khi không có barometer**, và hướng giải quyết nhìn chung tốt hơn nhiều so với việc tiếp tục dùng ngưỡng Pitch.

Nhưng có một vấn đề quan trọng: **một số giả định sinh học/động học trong bản hiện tại đang được biến thành “quy tắc cứng” hơi sớm**. Nếu triển khai nguyên xi, khả năng cao hệ thống sẽ chạy đẹp trên 2 tập replay hiện tại nhưng giảm độ tin cậy trên người dùng khác, cách cầm điện thoại khác, tốc độ bước khác.

Mình sẽ chấm bản này khoảng **8.4/10 ở ý tưởng**, nhưng chỉ khoảng **7/10 ở mức sẵn sàng để đóng thành production implementation**.

---

# 1. Điểm đúng nhất của đề án: bỏ Pitch tuyệt đối

Đây là quyết định rất đúng.

Tài liệu đã xác định rõ vấn đề: điện thoại cầm tay có thể giữ Pitch khoảng `-15° → -25°` ở cả hai chiều, nên suy diễn “điện thoại chúi lên = leo cầu thang” là sai mô hình. 

Cho nên kiến trúc mới:

```text
Orientation-independent vertical dynamics
                ↓
Temporal gait features
                ↓
Step-wise vertical evidence
                ↓
Ascent / Descent inference
```

tốt hơn rất nhiều so với:

```text
phone pitch
    ↓
if > threshold => ascent
```

Đặc biệt, việc đưa gia tốc về Earth frame trước khi trích đặc trưng là bước kiến trúc quan trọng:

$$
a_{world}=R(q)a_{body}-g\hat z
$$

Tài liệu cũng đã xác định đúng điều này trong pipeline. 

---

# 2. Tuy nhiên: “Skewness = hướng cầu thang” không nên được coi là luật vật lý tuyệt đối

Đây là điểm mình muốn chỉnh mạnh.

Tài liệu đang đặt giả định:

```text
positive skewness → ascent
negative skewness → descent
```



và sau đó đưa thẳng vào evidence:

```cpp
ascent_evidence =
    0.40 * skewness +
    ...
```



Vấn đề là skewness của gia tốc **rất phụ thuộc cách người dùng cầm điện thoại**:

* cầm trước ngực;
* cầm ngang;
* bỏ túi quần;
* cầm bằng một tay;
* vung tay;
* giữ máy cố định;
* vừa nhìn màn hình vừa đi;
* đi nhanh/chậm.

Thậm chí cùng một người, cùng cầu thang, chỉ cần thay đổi kiểu cầm là waveform có thể thay đổi đáng kể.

### Giải pháp tốt hơn

Đừng dùng:

```text
sign(skewness) → direction
```

mà dùng:

```text
skewness → likelihood
```

Tức là:

$$
P(A|F_{skew})
$$

và

$$
P(D|F_{skew})
$$

Skewness chỉ là **một sensor/evidence channel**, không phải ground truth.

---

# 3. ZUPT Vertical Drift là thành phần mạnh nhất — nhưng hiện tại đang được mô tả hơi lạc quan

Tài liệu đề xuất:

$$
v_z[k]=v_z[k-1]+a_z[k]\Delta t
$$

sau đó reset:

$$
v_z \rightarrow 0
$$

ở stance phase, rồi lấy:

$$
\Delta z_{step} = \int v_z dt
$$



Ý tưởng này rất mạnh.

Nhưng có một vấn đề:

### ZUPT không tự nhiên biến việc tích phân IMU thành phép đo độ cao chính xác.

Sai số bias:

$$
a_z = a_{true}+b_a+n
$$

sau tích phân một lần:

$$
v_z \approx v_{true}+b_at
$$

sau tích phân lần hai:

$$
z \approx z_{true}+\frac12b_at^2
$$

Do đó chỉ cần bias nhỏ cũng làm `Δz_step` drift khá nhanh.

### Cách mình khuyên

Không dùng `step_delta_z` như:

> “đo trực tiếp độ cao của một bậc”.

Thay vào đó dùng nó như:

> **vertical-motion evidence**.

Tức:

```text
Δz_step > 0
→ evidence ascent tăng

Δz_step < 0
→ evidence descent tăng
```

nhưng phải gắn:

```text
confidence_z
```

vào.

---

# 4. Nên biến `StairDetectorFeatures` thành Feature + Confidence

Hiện tại struct:

```cpp
rise_time
peak_accel
trough_accel
freefall_duration
waveform_skewness
step_delta_z
vertical_jerk
ascent_score
descent_score
```



là hợp lý.

Nhưng mình sẽ nâng cấp thành kiểu:

```cpp
struct StairDetectorFeatures {
    float waveform_skewness{};
    float freefall_duration_ms{};
    float step_delta_z_m{};
    float vertical_jerk_m_s3{};

    float cadence_hz{};
    float step_period_s{};
    float stance_ratio{};
    float swing_ratio{};

    float imu_quality{};
    float orientation_confidence{};
    float zupt_confidence{};
    float step_confidence{};

    float ascent_likelihood{};
    float descent_likelihood{};
};
```

Điểm khác biệt rất lớn:

### hiện tại

```text
feature = number
```

### nên thành

```text
feature = number + confidence
```

Đây là cách để hệ thống sống được khi:

* sensor noise tăng;
* orientation estimate kém;
* người dùng chạy;
* cầm điện thoại bất thường;
* bước chân không đều.

---

# 5. “Multi-Indicator Bayesian Fusion” hiện tại chưa thực sự Bayesian

Đây là một điểm về kiến trúc/tên gọi.

Tài liệu gọi:

> Multi-Indicator Bayesian Fusion

nhưng implementation thực tế lại là:

```cpp
0.40 * A
+ 0.40 * B
+ 0.20 * C
```



Đây là **weighted evidence fusion**, chưa phải Bayesian inference đúng nghĩa.

Không có:

$$
P(A|F)
\propto
P(F|A)P(A)
$$

hay likelihood model / posterior update.

### Mình khuyên hai lựa chọn:

#### Cách 1 — đơn giản và production-friendly

Đổi tên thành:

```text
Weighted Evidence Fusion
```

và giữ trọng số.

#### Cách 2 — nâng cấp thật

Dùng:

$$
L_A = \log P(F_1|A)+\log P(F_2|A)+...+\log P(A)
$$

$$
L_D = \log P(F_1|D)+\log P(F_2|D)+...+\log P(D)
$$

rồi softmax:

$$
P(A)=\frac{e^{L_A}}{e^{L_A}+e^{L_D}}
$$

Cách này đẹp hơn rất nhiều nếu sau này bạn thu được dataset thật.

---

# 6. Không nên hard-code “0.17 m/bước” làm độ cao thật

Đây là điểm mình sẽ sửa **ngay**, dù nó rất tiện.

Tài liệu hiện tại:

```text
Ascent → +0.17 m/bước
Descent → -0.17 m/bước
```



Về mặt mô phỏng cầu thang cụ thể thì ổn.

Nhưng về mặt engine:

```text
0.17 m
```

không phải đặc tính của mọi cầu thang.

Có thể:

```text
0.14
0.16
0.18
0.20
0.21
```

và còn thay đổi theo người dùng.

### Tốt hơn:

Tách:

```text
Step Direction
```

ra khỏi:

```text
Stair Geometry Estimation
```

Ví dụ:

```text
direction = ascent/descent
height_increment_est = estimated_riser_height
```

và:

$$
h_{step}
$$

được ước lượng từ dữ liệu.

Ví dụ:

```text
prior = 0.15–0.20 m
measurement = vertical-motion evidence
posterior → estimated riser
```

Khi chưa đủ dữ liệu:

```text
use prior
```

Không nên giả định 17 cm là chân lý toàn hệ thống.

---

# 7. Chiếu nghỉ không nên chỉ dựa vào “xoay 180°”

Đoạn:

> khóa theo vế thang 8–12 bậc, phát hiện chiếu nghỉ qua xoay 180° hoặc landing candidate. 

Ý tưởng tốt nhưng có rủi ro.

Không phải người dùng nào cũng:

```text
đi 10 bậc
→ đúng 180°
```

Có thể:

```text
160°
175°
185°
200°
```

hoặc chiếu nghỉ dài rồi mới quay.

### Tốt hơn:

Tạo một:

## `StairFlightLikelihood`

được tính từ:

```text
step continuity
+
heading change
+
cadence continuity
+
vertical direction continuity
+
horizontal displacement
+
turn duration
```

Thì:

```text
heading change ≈ 180°
```

chỉ là **một feature**.

---

# 8. FSM hiện tại đúng hướng nhưng nên trở thành HMM/Probabilistic FSM

Hiện tại:

```text
≥ 3 bước giống nhau
→ lock state
```



đây là anti-flicker tốt.

Nhưng:

```text
3 bước
```

là một magic number.

Mình sẽ thiết kế:

```text
CANDIDATE_ASCENT
CANDIDATE_DESCENT
CONFIRMED_ASCENT
CONFIRMED_DESCENT
LANDING
TRANSITION
UNKNOWN
```

và mỗi transition phụ thuộc:

```text
confidence
+
consecutive steps
+
contradictory evidence
```

Ví dụ:

```text
UNKNOWN
  ↓ evidence > 0.60
CANDIDATE_ASCENT
  ↓ evidence > 0.75 for 3 steps
CONFIRMED_ASCENT
```

Nếu evidence giảm:

```text
CONFIRMED_ASCENT
  ↓ confidence < 0.45 for 2 steps
UNKNOWN
```

Như vậy không cần “hard lock” quá cứng.

---

# 9. Một vấn đề rất quan trọng mà tài liệu chưa giải quyết: người dùng có thể dừng giữa cầu thang

Ví dụ:

```text
Ascent
Ascent
Ascent
STOP
look at phone
continue
```

Nếu FSM coi stop là mất stair state:

```text
stair → walking
```

rồi bước tiếp theo phải nhận diện lại.

Điều này gây flicker.

Nên có:

```text
STAIR_PAUSED
```

hoặc:

```text
TEMPORARY_STANCE
```

với timeout:

```text
0–2 s → giữ stair hypothesis
2–5 s → giảm confidence
>5 s → release
```

---

# 10. Một điểm còn thiếu: stair ascent/descent phải được tách khỏi “stair detection”

Hiện tại kiến trúc có vẻ đang làm:

```text
stairs?
    +
up/down?
```

nhưng hai bài toán này nên tách:

$$
P(\text{stairs})
$$

và:

$$
P(\text{ascent}|\text{stairs})
$$

và:

$$
P(\text{descent}|\text{stairs})
$$

Tức:

```text
              ┌───────────────┐
IMU ─────────>│ Stairness     │
              └──────┬────────┘
                     │
                  P(stair)
                     │
          ┌──────────┴──────────┐
          ▼                     ▼
    P(ascent|stair)       P(descent|stair)
```

Đây là kiến trúc tốt hơn nhiều.

Vì:

```text
negative skewness
```

không có nghĩa:

> descent

nếu thực tế đó là:

> đi bộ bình thường + va tay.

---

# 11. Mình đặc biệt không thích acceptance criterion “phân loại đúng 100%”

Tài liệu đề xuất:

> góc nghiêng điện thoại -25° → phân loại đúng 100%. 

Đây là test tốt để kiểm regression của scenario đó, nhưng **không nên coi 100% là acceptance criterion của classifier**.

Classifier thực nên đánh giá:

```text
accuracy
precision
recall
F1
false ascent rate
false descent rate
switch latency
```

Đặc biệt:

### metric đáng quan tâm nhất

```text
False Ascent Rate
False Descent Rate
```

vì trong hệ thống navigation:

> nhận nhầm walking thành stair descent

có thể nguy hiểm hơn việc đôi lúc trả `UNKNOWN`.

---

# 12. Tập real-world 2 phiên là quá nhỏ để tuning

Hiện tại có:

```text
79.4 s / 145 steps
66.1 s / 105 steps
```



Đủ để debug.

Chưa đủ để “validate”.

Mình sẽ chia dataset thành:

```text
Train / Tune
Validation
Blind Test
```

và đặc biệt phải có:

### người dùng khác.

Không chỉ:

```text
same person
same phone
same stairwell
same route
```

Mà:

```text
User A / Phone A
User B / Phone A
User A / Phone B
User B / Phone B
different staircases
different cadence
different grip
```

Nếu không, rất dễ xảy ra:

> overfit người dùng + overfit cầu thang.

---

# 13. Kiến trúc mình đề xuất thay cho bản hiện tại

Mình sẽ xây nó thành **Stair Vertical Motion Engine**:

```text
                IMU
                 │
                 ▼
       ┌──────────────────┐
       │ Sensor Integrity  │
       └────────┬─────────┘
                ▼
       ┌──────────────────┐
       │ Orientation/ESKF │
       └────────┬─────────┘
                ▼
       Earth-frame accel
                │
        ┌───────┴────────┐
        ▼                ▼
 ┌──────────────┐  ┌──────────────┐
 │ Step Detector│  │ Stairness    │
 └──────┬───────┘  └──────┬───────┘
        │                 │
        ▼                 │
 ┌──────────────┐          │
 │ ZUPT Engine  │          │
 └──────┬───────┘          │
        │                  │
        ▼                  │
 ┌──────────────┐          │
 │ Step Features│◄─────────┘
 └──────┬───────┘
        ▼
 ┌────────────────────────────┐
 │ Direction Evidence Engine  │
 │                            │
 │ skewness                   │
 │ vertical displacement      │
 │ freefall                   │
 │ impact                     │
 │ cadence                    │
 │ ZUPT confidence            │
 └─────────────┬──────────────┘
               ▼
     ┌──────────────────────┐
     │ Probabilistic Fusion │
     └──────────┬───────────┘
                ▼
     ┌──────────────────────┐
     │ Stair Flight FSM     │
     └──────────┬───────────┘
                ▼
       Navigation Estimate
```

Đây là kiến trúc mình nghĩ hợp với engine hiện tại nhất.

---

# 14. Cụ thể hóa scoring

Thay vì:

```cpp
0.4*skew + 0.4*z + 0.2*score
```

mình đề xuất chuẩn hóa tất cả thành likelihood:

```text
F1 = skewness likelihood
F2 = Δz likelihood
F3 = freefall likelihood
F4 = impact timing likelihood
F5 = cadence likelihood
F6 = step regularity
F7 = ZUPT confidence
```

Sau đó:

$$
L_A =
w_1F_{1A}
+w_2F_{2A}
+\cdots
+w_7F_{7A}
$$

$$
L_D =
w_1F_{1D}
+w_2F_{2D}
+\cdots
+w_7F_{7D}
$$

rồi:

$$
P_A = softmax(L_A,L_D)
$$

$$
P_D = softmax(L_A,L_D)
$$

và:

$$
P_U=1-P_A-P_D
$$

`UNKNOWN` rất quan trọng.

---

# 15. Thêm “direction hysteresis”

Ví dụ:

```text
P(ascent) = 0.78
P(descent) = 0.15
```

→ ascent.

Nhưng đừng đổi sang descent chỉ vì một frame:

```text
P(ascent)=0.48
P(descent)=0.51
```

Thay vào đó:

```text
confirmed ascent
→ cần P(desc) > 0.75
→ trong N consecutive steps
```

Đây sẽ loại bỏ cực nhiều false flip.

---

# 16. Cách ước lượng độ cao nên thay đổi

Hiện tại:

```text
step × 0.17m
```

Mình đề xuất:

```text
direction_estimate
        +
riser_estimate
        +
confidence
```

Ví dụ:

$$
h_{riser}=h_{prior}+\Delta h_{measurement}
$$

và giới hạn:

$$
h_{min}<h_{riser}<h_{max}
$$

Sau vài bậc:

```text
step 1 → 0.15
step 2 → 0.17
step 3 → 0.18
...
posterior ≈ 0.17
```

thì engine tự hội tụ.

Đây chính là chỗ hệ thống của bạn có thể tiến hóa từ “rule-based” thành **probabilistic terrain inference**.

---

# 17. Điểm cực hay: có thể dùng chính Flight FSM để tự hiệu chỉnh

Giả sử phát hiện:

```text
10 steps
```

và tổng vertical displacement estimate:

```text
1.64 m
```

thì:

$$
h_{riser}\approx0.164m
$$

Khi tiếp tục vế thứ hai:

```text
1.68m / 10
```

→ estimate ~0.168m.

Sau đó engine học:

```text
estimated riser ≈ 0.166m
```

Điều này mạnh hơn hard-code rất nhiều.

---

# 18. Test suite mình khuyên bổ sung

Không chỉ 4 test trong tài liệu.

Hãy làm khoảng:

### Level 1 — Feature tests

```text
Skewness calculation
ZUPT
freefall detector
landing detector
step period
```

### Level 2 — Direction tests

```text
clean ascent
clean descent
slow ascent
fast ascent
slow descent
fast descent
```

### Level 3 — device pose

```text
phone -25°
phone -15°
phone 0°
phone +15°
phone +30°
```

### Level 4 — usage style

```text
handheld
pocket
chest
side grip
one-handed
two-handed
```

### Level 5 — adversarial

```text
flat walk + arm swing
running
jumping
vehicle vibration
rough road
abrupt stop
turning
phone rotation
```

### Level 6 — environmental

```text
stairwell airflow
metal stairs
concrete stairs
long landing
different riser heights
spiral stair
```

### Level 7 — sensor faults

```text
baro absent
IMU noise
IMU bias
IMU dropout
timestamp jitter
orientation degradation
```

Đây mới là bộ test có khả năng nói rằng thuật toán thực sự bền.

---

# 19. Một thứ còn rất đáng làm: Synthetic Physics Generator

Thay vì chỉ lấy dữ liệu thật, tạo simulator:

```text
StairState
    ├─ ascent/descent
    ├─ step height
    ├─ tread depth
    ├─ cadence
    ├─ phone orientation
    ├─ acceleration noise
    ├─ gyro noise
    └─ user style
```

rồi sinh waveform:

```text
ideal IMU
   ↓
sensor model
   ↓
noise
   ↓
bias
   ↓
dropout
   ↓
StairMotionDetector
```

Có thể chạy:

```text
10,000
50,000
100,000
```

trajectory Monte Carlo.

Lúc đó bạn có thể tìm:

```text
threshold tối ưu
weight tối ưu
FSM transition tối ưu
```

thay vì tự chọn bằng tay.

---

# 20. Lộ trình implementation mình khuyên

Không nên làm theo thứ tự file trong tài liệu. Mình sẽ làm như sau:

## Phase A — Stabilize signal layer

```text
Earth-frame acceleration
↓
gravity compensation
↓
orientation confidence
↓
timestamp validation
```

Mục tiêu: đảm bảo `a_z` đáng tin trước.

---

## Phase B — Step segmentation

Triển khai:

```text
StepDetector
├── swing
├── stance
├── landing
├── cadence
└── step confidence
```

Không bàn ascent/descent ở phase này.

---

## Phase C — Vertical motion estimator

Triển khai:

```text
ZUPTVelocityEstimator
```

Output:

```text
v_z
Δz_step
confidence_z
```

---

## Phase D — Stairness classifier

Trả về:

```text
P(stairs)
P(flat_walk)
P(other)
```

Chưa phân biệt ascent/descent.

---

## Phase E — Direction classifier

Chỉ khi:

```text
P(stairs) > threshold
```

mới kích hoạt:

```text
ascent vs descent
```

---

## Phase F — Flight FSM

Quản lý:

```text
candidate
confirmed
pause
landing
transition
```

---

## Phase G — Vertical map update

Cuối cùng mới làm:

```text
Z += estimated_riser
```

và truyền vào Particle Filter.

Đây là thứ tự mình tin cậy hơn hẳn việc cùng lúc sửa classifier + FSM + Z.

---

# 21. Và đây là điểm nối cực quan trọng với v2.3 trước

Hai bản kế hoạch này thực ra đang bắt đầu ghép thành **một hệ thống rất đẹp**.

Bản trước:

```text
Vehicle propagation
+
Particle Filter
+
Resource Governor
+
Integrity
```

Bản hiện tại:

```text
Stair vertical inference
+
Flight FSM
+
ZUPT
+
Ascent/Descent probability
```

Khi ghép lại:

```text
                Sensor
                  ↓
             Integrity
                  ↓
             ESKF/PDR
                  ↓
        ┌─────────┴─────────┐
        ↓                   ↓
 Motion Activity       Stair Engine
        ↓                   ↓
 Walking/Vehicle       Up / Down
        │                   │
        └─────────┬─────────┘
                  ↓
          Continuous PF
                  ↓
        Terrain Constraints
                  ↓
        Rescue Probability
```

Đây mới là kiến trúc rất mạnh.

---

# 22. Mình sẽ thay đổi acceptance criteria thành thế này

Không dùng:

```text
“đúng 100%”
```

mà dùng:

### Direction classification

```text
F1 ≥ 0.95
False ascent < 1%
False descent < 1%
direction switch latency ≤ 2 steps
```

### Stair detection

```text
Precision ≥ 95%
Recall ≥ 95%
```

### Vertical estimate

```text
per-flight error ≤ X m
```

### Flight FSM

```text
0 false direction flips / flight
```

### Robustness

```text
phone orientation perturbation ±30°
sensor noise ±N
timestamp jitter ±M ms
```

Các con số cuối cùng nên được xác định từ dataset, chứ không nên tùy ý đặt tất cả từ đầu.

---

# 23. Kết luận thẳng

Mình **ủng hộ triển khai ý tưởng này**, đặc biệt ba thứ:

**Earth-frame vertical acceleration → step-wise ZUPT → multi-evidence direction inference.**

Ba lớp này là nền tảng đúng.

Nhưng mình sẽ **không merge nguyên bản** vì có 5 điểm cần sửa:

| Thành phần      | Bản hiện tại            | Mình khuyên                           |
| --------------- | ----------------------- | ------------------------------------- |
| Pitch           | bỏ ngưỡng               | ✅ giữ                                 |
| Skewness        | evidence mạnh           | ✅ giữ nhưng giảm vai trò              |
| ZUPT Δz         | gần như direction truth | → dùng kèm confidence                 |
| Bayesian        | weighted sum            | → likelihood fusion thật hoặc đổi tên |
| 0.17m/step      | hard-code               | → adaptive riser estimate             |
| 180° landing    | rule                    | → heading-change likelihood           |
| 3 bước lock     | hard threshold          | → confidence + hysteresis             |
| 100% accuracy   | acceptance              | → F1/precision/recall/false-rate      |
| 2 replay files  | validation              | → chỉ debugging; cần blind test       |
| Stair direction | gộp chung               | → tách Stairness và Direction         |

### Định hướng cuối cùng mình khuyên:

**Đừng xây một “Ascent/Descent Detector”. Hãy xây một `Probabilistic Stair Motion Engine`.**

Đầu ra của engine không nên chỉ là:

```cpp
kStairsAscent
```

mà nên là:

```cpp
struct StairMotionState {
    StairActivity activity;       // none / stair
    float p_stair;
    float p_ascent;
    float p_descent;

    float step_height_m;
    float vertical_displacement_m;

    float step_confidence;
    float direction_confidence;
    float flight_confidence;

    uint32_t flight_step_count;
    FlightPhase phase;
};
```

Khi đó **classifier, FSM, PDR, Particle Filter và Rescue Engine đều có thể dùng cùng một probabilistic state**, thay vì mỗi module tự “đoán” một lần.

Đấy mới là bước mình nghĩ sẽ đưa OFF-GRID SURVIVAL từ một navigation prototype mạnh lên thành **một probabilistic navigation engine thực sự**. 
