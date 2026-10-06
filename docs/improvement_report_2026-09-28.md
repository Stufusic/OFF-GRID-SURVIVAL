# BÁO CÁO CẢI TIẾN TOÀN DIỆN — OFF-GRID SURVIVAL ENGINE

**Ngày:** 2026-09-28 | **Phạm vi:** Bản cập nhật 3 — Cập nhật thuật toán & độ chính xác (A1–A7)
**Đánh giá trên:** 2 dataset tự thu (Infinix X6837, Sensor Logger 100Hz) + MotionSense (24 người, 360 recordings, 50Hz, nhãn hoạt động người-gán)

---

## 1. KẾT QUẢ TRƯỚC / SAU

### 1.1 MotionSense 360 recordings (nhãn hoạt động thật — ground truth độc lập)

| Chỉ số | Baseline | Sau cải tiến | Thay đổi |
|---|---|---|---|
| Độ chính xác hoạt động trội (tổng) | 39.4% | **41.9%** | +2.5đ |
| Nhận đúng ĐI BỘ | 1.4% | **65.3%** | **+63.9đ** |
| Nhận đúng XUỐNG thang | 84.7% | 40.3% | −44.4đ (xem §3) |
| Nhận đúng CHẠY | 0.0% | 27.1% | +27.1đ |
| Nhận đúng LÊN thang | 0.0% | 0.0% | không đổi (xem §3) |
| Đứng yên | 83.3% | 64.6% | −18.7đ (bước ma 1Hz do artifact dataset — §3) |
| Sai số đếm bước (median, lớp di chuyển) | −1.6%…−4.6% | **−1.0%…+1.1%** | cải thiện |

### 1.2 Hai dataset tự thu (tham chiếu vật lý đã xác nhận của dự án)

| Chỉ số | D1 (buồng thang) | D2 (jog loop) |
|---|---|---|
| Số bước | 124 (−14.5%) → **126 (−13.1%)** | 98 (+17% tạm) → **108 (+8.0%)** |
| Loop closure ngang | 10.95m → **6.14m** (mục tiêu ≤7 ✓) | 23.91m → **8.09m** (C3b) |
| Cao độ cuối (thật ~0m) | −14.85m → **−0.17m** (✓ mục tiêu ≤2m, C3b) | −12.70m → 0.00m |
| Hướng leo pha LÊN bị gán "descend" | 100% sai → vẫn sai (IMU-only không đủ — §3) | — |

### 1.3 Chất lượng kỹ thuật

| Hạng mục | Trước | Sau |
|---|---|---|
| Unit test | 182/182 | **183/183** (thêm regression dữ liệu thật 16 recordings, tự SKIP khi thiếu data) |
| Tool đánh giá | build tay ngoài CMake, output mất im lặng khi mở file lỗi | Target CMake chính thức + argv + CSV machine-readable + kiểm tra `is_open()` |
| Benchmark | script one-off regex stdout | `run_benchmarks.py` thống nhất (CSV baseline/phase) + `check_all.sh` |

---

## 2. CÁC LỖI GỐC ĐÃ TÌM RA VÀ SỬA (đúng thứ tự quan trọng)

### [C1-a] ESKF tilt không bao giờ hội tụ khi chuyển động — GỐC RỄ LỚN NHẤT
`eskf_3d.cpp` chỉ hiệu chỉnh roll/pitch khi quasi-static. Điện thoại nghiêng tĩnh
45–65° trong túi + đi liên tục → tilt kẹt ở khởi tạo → **mọi trục đứng phía hạ lưu
sai**: a_v_dynamic mang bias −4…−10 m/s² (đo: với trục gravity thật mean ≈ 0, với
"ESKF giữ mức" mean ≈ −10.5), delta-Z tích phân phân kỳ −0.6…−19 m/bước, hướng
luôn "descend", guard chống đi-phẳng vô hiệu, p_stair=0.98 cả khi đi bộ.
**Fix:** nhánh tilt liên tục motion-tolerant (tham chiếu gravity LP chậm τ=2.5s,
gain × motion_gain, warm-boost 3s đầu) + `-Wa,-mbig-obj` cho file ESKF.

### [C1-b] Pitch tuyệt đối ±0.08 rad vô nghĩa khi máy nghiêng tĩnh
Pitch nền tự hiệu chuẩn bằng EMA khi đứng yên; mọi ngưỡng pitch (classifier +
pipeline stair gate) chuyển sang **relative pitch**.

### [C1-c] Kênh hướng của StairMotionDetector sai cực tính trên dữ liệu thật
Audit probe (docs/probe_*.csv) trên 11 nhóm.truth: **freefall kích hoạt trên ~100%
bước đi bộ thường** (đáy sải chân) — luôn bầu "descend"; delta-Z tích phân
reset-mỗi-bước mang DC bias cấu trúc; p2p của đi bộ túi (23–35) LỚN HƠN thang
(13–19) — giả định cũ ngược thực tế. **Fix:** p_stair dải p2p hai phía [3,16–26]
bỏ kênh delta-Z; softmax hướng đặt skewness làm chủ đạo (kênh duy nhất có cực
tính nhất quán); bỏ bonus freefall.

### [C1-d] FSM khóa trạng thái + classifier dìm WALKING/RUNNING
kConfirmed bỏ qua p_stair thấp vĩnh viễn → thêm lối thoát 4 bước p_stair thấp;
candidate thang phải qua cửa bằng chứng DUY TRÌ (≥5/8 bước p_stair ≥ 0.60);
điểm thang không sustained bị triệt zero trước khi nạp FSM ngoài; RUNNING ưu
tiên theo nhịp (cadence 2.3+ không bị FSM thang chặn); cadence lọc median 3 bước.

### [C2] Lọc bước: BandpassFilter cứng Fs=100Hz (50Hz bị méo dải) + double-peak
Bandpass tự thích ứng Fs (RBJ biquad, thiết kế lại khi lệch >8%); luật chống
double-peak theo TỈ LỆ (chặn khoảng bước < 0.65×chu kỳ trung bình, trần 0.45s) —
harmonics sải chân sau khi tilt đúng tạo đỉnh giả cách 0.35–0.40s (D2: 28/121).

### [C3] ZARU chết — giờ được gọi khi quasi-static (cổng tĩnh sâu |ω−bg|<0.02)
D2: đếm bước 100/100 chính xác tuyệt đối trong lần thử, loop closure giảm sâu.

### [C3b] Mag update heading-pure (bản cập nhật 2) — cải tiến lớn nhất đợt này
K = −P.col(2)/S cũ cho phép innovation **heading** kéo cả roll/pitch/bias-x/y
qua coupling hiệp phương sai — nhiễu từ tạo wobble tilt làm rung mọi nhãn phía
hậu. **Fix:** chiếu K về không gian con {yaw, bias-gyro-Z} trước khi bơm.
Nhờ đó mở lại **mag-recovery** (+0.06/accept sau reject) một cách an toàn.
Kết quả thực đo: **D2 loop closure 20.10 → 8.09m (−60%)**, **D1 cao độ cuối
−2.31 → −0.17m**, hai dataset khác không bị suy giảm.

### [C3c] Hiệu chỉnh hiệp phương sai PDR (rvcore.md [MEDIUM] NEES 320)
Đo Monte Carlo bằng probe (nees_probe): Pure PDR **NEES = 357.6**, Full Fusion
12.6. Hiệu chỉnh: σ0 sải bước 0.04 → 0.12 m; floor heading variance 0.01 →
0.02 rad²; thổi phồng heading variance ×2.0 khi đang trên cầu thang. Kết quả:
**Pure PDR NEES 357 → 206**, Full Fusion ổn định 12.5, vị trí trung bình trên
dữ liệu thật không đổi (variance chỉ ảnh hưởng bất định).

---

## 3. GIỚI HẠN ĐÃ XÁC ĐỊNH BẰNG DỮ LIỆU (không phải việc chưa làm)

1. **Hướng lên/xuống IMU-only là bài toán chưa có nghiệm trên dữ liệu túi 50Hz:**
   cả 5 kênh per-step (p2p, v2h, rise, skew, delta-Z kể cả tích phân khử-mean)
   đều chồng lấn giữa up/down/walking khi xét từng file. Skewness là kênh duy
   nhất có cực tính đúng (D1: −0.09 vs −0.29; D2: −0.01 vs −0.10; MS: +0.17 vs
   +0.02) nhưng lệch theo vị trí đặt máy. **Barometer là nguồn hướng duy nhất
   đáng tin** — khuyến nghị bắt buộc bật Watch Barometer khi thu dữ liệu.
2. **Artifact MotionSense:** gravity CoreMotion cập nhật ~1Hz → bậc thang giả
   trong tổng accel → ~250 "bước" 1.00Hz khi ngồi (đã thử LP gravity — tệ hơn
   tổng thể, đã revert và ghi chú trong converter).
3. **Đứng yên 64.6%:** do chính artifact (2) — các lượt sit/std có bước ma.
4. D1 bước −11.7% (was −14.5%): bộ tham chiếu 145 bước của Python đếm cả bước
   landing; trade-off với loop closure 6.14m được chấp nhận có chủ đích.

---

### [C4] Giai đoạn 4 — cấu hình hóa & vệ sinh (bản cập nhật 2)

- **`MotionActivityClassifierConfig` / `StairMotionDetectorConfig`**: các ngưỡng
  then chốt đã hiệu chỉnh (pitch gate, baro bands, cửa sustain, dải cadence,
  hệ số p_stair, trọng số hướng hướng, FSM confirm/anti-flip/exit) đưa vào 2
  config struct với default = giá trị đã tune. Hành vi không đổi (benchmark
  khớp bit-per-bit, 183/183 test).
- **Baro stale expiry**: classifier dùng rate EMA cũ vô thời hạn khi baro rời
  mẫu — thêm expiry 2s đồng bộ với zero-order-hold của pipeline.
- **M2 criterion tương đối** (rvharness.md [MEDIUM]): `passes_m2_baro_drift_criteria`
  chạy baseline M0 ghép cặp cùng seed nội bộ, nghiệm thu CE95 ≤ 1.2×M0 (kèm trần
  an toàn 150m) — đúng đặc tả header thay vì hằng số tuyệt đối 120m cũ.
- **validate_packet.py**: thêm self-test biên ±90/±180°, Everest/đáy biển,
  CE biên — oracle chéo dùng được trong CI (`check_all.sh` bước 4).
- Dọn warning `seg_start_t` unused trong replay_evaluation.

## 3b. NÂNG CẤP UI/UX + DASHBOARD (Dear ImGui, hoàn thành cùng ngày)

Desktop app **viết lại hoàn toàn** từ monolith Win32/GDI+ 517 dòng sang
**Dear ImGui v1.90.9-docking + ImPlot v0.16, backend Win32 + DirectX11**
(FetchContent, không thêm GLFW; WARP fallback khi không có GPU):

| Thành phần | Nội dung |
|---|---|
| Bản đồ Tactical | grid UTM 250m, contour núi, PF cloud (culling viewport), **4 cụm giả thuyết + ellipse CE95 riêng từng cụm** (trước chỉ 1), trail 500 điểm, **zoom scroll / pan chuột / fit-all** |
| Telemetry | 6 chỉ số lớn (GNS/CE95/bước/quãng đường/cao độ/pin) + **ImPlot time-series 4 đường @10Hz ring 60s** + stair state (riser/landings) + ma trận SystemFlags phân loại OK/informational |
| An toàn | 4 trạng thái MissionState màu + pulse animation + nút Hủy báo động + bằng chứng cảm biến (cadence/impact/freefall) |
| Cứu hộ | gói 160-bit hex + SMS live, **nút Copy clipboard**, GeoOrigin chỉnh được (bỏ hardcode Hà Nội), lịch sử 10 gói |
| Controls | nút + slider thay phím tắt, FPS thật, pacing bước theo wall-clock |
| **Replay (U2)** | phát lại dataset thật: preset 2 dataset tự thu + 64 recordings MotionSense từ manifest, **play/pause/step/tốc độ 0.5–8×**, thanh tiến trình, **timeline hoạt động tô màu 1px=1s**, loop closure/CE95 trực quan |

Sửa kèm các bug review của GUI cũ: gyro sai đơn vị (sim bơm rate qua 55 mẫu
100Hz/bước với dạng sóng "đẩy sớm" đã audit — classifier giờ nhận đúng DI BO
cadence 1.82Hz ngay trong dashboard), phím G chết → GPS handoff thật qua
`updateAnchorPosition()`, WIN32_EXECUTABLE, SMS UTF-8, map clipping.

Test: `tests/test_desktop_hud.cpp` — 8 test toán thuần (MapView round-trip/pan/
zoom-keep-cursor/fit, ellipse CE95 heading rotation, RingBuffer FIFO, GeoOrigin,
hex dump). **Toàn dự án: 191/191 test pass**, benchmark engine bất biến.
Kiểm chứng trực quan: `docs/dashboard_screenshot2.png`.

## 3c. CẬP NHẬT THUẬT TOÁN + ĐỘ CHÍNH XÁC (bản cập nhật 3)

### Trước / sau đợt này (đo `algorithms_final` vs `phase34_final`)

| Chỉ số | Trước | Sau | Ghi chú |
|---|---|---|---|
| D1 loop closure | 6.14m | **4.53m** | ref 2.54m |
| D1 Z_end | −0.17m | −4.46m* | *đỉnh +1.98m ở cấu hình a6c — trade-off với landing/step |
| D2 loop closure | 8.09m | **3.06m** | ref 2.06m |
| D2 Z_end | 0.00 (mất vertical) | **−1.81m** | stairs track hoạt động lại |
| D2 bước | +8.0% | **−5.0%** | |
| MS ĐỨNG YÊN | 64.6% | **100%** (96/96) | cổng p2p chống bước ma |
| MS tổng thể | 41.9% | 51.9% | |
| MS WALKING | 65.3% | 68.1% | |
| Test | 191 | **196/196** | +K-factor, +anchor gate, +golden cập nhật |

### Các thuật toán mới/sửa (A1–A5)

1. **[A1] Cổng chống bước ma**: sàn p2p 0.5 m/s² cho bước thường + re-gate 0.8
   cho nhánh resume-sau-nghỉ (trước chấp nhận MỌI đỉnh) → đứng yên MotionSense
   64.6% → 100%. Fix dead-code: ×2 heading variance trên thang kiểm tra activity
   TRƯỚC khi gán (không bao giờ chạy). PF: bỏ slope/aspect khi gradient DEM
   không hợp lệ; heading noise wrap circular. Thống nhất hằng số tread 0.28.
2. **[A5] Fix bế ngưỡng FSM landing**: kLandingTransition không có đường về
   kConfirmed — sau landing đầu tiên hệ thống kẹt, thang tracking sụp. Đây là
   fix có tác động lớn nhất đợt này (D1 loop 8.2 → 2.85m ngay lập tức).
3. **[A5] Landing phiên thang**: khoảng walking ≤ 8s giữa 2 lần vào vế thang =
   chiếu nghỉ (recordLandingStep hồi tố, chống đếm kép).
4. **[A6-thuật toán] Đảo hướng TƯƠNG ĐỐI trong phiên thang**: skewness tuyệt đối
   lệch theo vị trí đặt máy nhưng trong 1 phiên lên→xuống độ lệch 2 pha ổn định
   (D1: −0.09 vs −0.29). Cluster EMA + 3 bước lệch liên tiếp → đảo hướng +
   `fsm.forceActivity()` đồng bộ (outer FSM cấm ascent↔descent trực tiếp).
   Kết quả: D1 t_up 0s → 37s, Z_end −14.36 → +1.98m (a6c).
5. **[A3] PF → PDR phản hồi có giới hạn** (flag `pdr_pf_feedback`, gain 0.02,
   Mahalanobis ≤ 9, 1 cụm + neff ≥ 30%) + **anchor gate** |Δ| ≤ 1000m (trước
   hard-overwrite không gate).
6. **[A4] Neo dọc**: anchor GPS với |Δalt| > 3m hấp thụ vào baro_bias (trước
   chuỗi baro đấu lại → trôi tái phát).
7. **[A2] Hiệu chuẩn stride K theo GNSS** (`StrideScaleEstimator` — EMA nhân
   tính đúng, outlier ratio gate, clamp [0.7, 1.4]) + 4 test cơ chế.

### Bài học đo đạc (khóa cấu hình)

- Nới cửa sustained (0.55) + p_lo ramp (4.0): nhãn stair 39s → 67s nhưng vertical
  DỮ (D1 Z_end +1.98 → −4.79) — walking-as-stairs trung gian. Giữ 0.60/6.0.
- FSM unlock stair-entry: cùng hệ quả — đã hoàn tác có ghi chú trong code.
- Landing đếm 1/18 (D1) dưới mục tiêu — window/nhãn nhưng không phá các chỉ
  số khác; để phiên sau với ML-lite.

## 3d. DASHBOARD WEB + UI (bản cập nhật 4)

Nhúng web server (cpp-httplib v0.15.3) vào desktop app — dashboard xem được trên
trình duyệt của máy hoặc **điện thoại cùng LAN** (opt-in). Chỉ-đọc, an toàn.

- **Endpoints**: `GET /` (UI), `/api/state` (JSON snapshot), `/api/events`
  (SSE ~4Hz, tự reconnect), `/api/health`.
- **Snapshot an toàn luồng**: main loop publish `WebState` mỗi frame dưới mutex;
  web thread phục vụ từ snapshot — vòng lặp ImGui không bao giờ bị chùng.
- **Web UI** (HTML/JS/CSS thuần, không build tooling): banner an toàn 4 trạng
  thái pulse, 6 cards chỉ số, **bản đồ tactical canvas** (grid/trail/PF cloud/4
  cụm CE95/vị trí thật-ước lượng, zoom + pan), **series chart 4 đường** tự vẽ,
  hoạt động + health flags, gói cứu hộ + copy. Theme AMOLED khớp desktop.
- **Kiểm chứng**: curl tất cả endpoints + **browser thực**: SSE "● trực tiếp",
  canvas render trail/PF/heading với dữ liệu live (screenshot lưu docs).
- **Test**: `test_web_json.cpp` — 5 unit test JSON writer (escape, trường bắt
  buộc, particles chỉ-alive, series). **Toàn dự án 201/201 test pass.**
- **Dùng**: chạy desktop app → panel "Web dashboard" → Start (hoặc
  `survival_desktop_gui.exe --web-port 8080`) → mở http://127.0.0.1:8080.

## 4. VIỆC CÒN LẠI

- `imu_health_` write-only và `PdrEngine` tích phân trùng (được 2 test dùng) —
  cần quyết định kiến trúc (xóa hay wire) trước khi dọn.
- Đáp ứng mag-recovery khi có dataset nhiễu từ thật; PF feedback (flag OFF)
  cần dữ liệu DEM thực để cân bằng gain.
- **A6 ML-lite** (chưa làm): học classifier offline từ 360 file đã nhãn — hướng
  đã có kênh tương-đối-in-phiên (A6-thuật toán); ML giúp walking/run/stationary.
- **D1 bước −14.5%** (mục tiêu −8%): các bước landing giữa vế vẫn bị nuốt.
- **Landing count** 1/18: cần kênh phát hiện landing độc lập (cadence-stability?).
- **Hướng thang MS túi quần 50Hz**: cần barometer (giới hạn vật lý đã chứng minh).
- **CI:** `check_all.sh` chạy local; khi lên git cắm GitHub Actions.

## 5. CÁCH CHẠY LẠI ĐÁNH GIÁ

```bash
scripts/check_all.sh          # build + ctest + benchmark subset 40 + oracle
scripts/run_benchmarks.py --label <tên> --full   # benchmark đầy đủ 360 recordings
python scripts/run_benchmarks.py --label phase3d  # số liệu mốc hiện tại
```
File số liệu: `docs/<tên>_summary.txt`, `docs/<tên>_metrics_*.csv`.
