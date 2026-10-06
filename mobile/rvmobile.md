# CODE REVIEW — THƯ MỤC `mobile/` (RENDERER ANDROID — HIỆN LÀ DEAD CODE)

**Ngày review:** 06/09/2026 | **Phạm vi:** `mobile/android/app/src/main/cpp/gui/` — `egl/fps_governor.*`, `renderer/ribbon_renderer.*`

---

## 1. TỔNG QUAN

Chỉ có 4 file: `FpsGovernor` (giới hạn tần số render theo trạng thái/pin) và `RibbonRenderer` (sinh mesh dải ruy-băng dọc quỹ đạo + fan ellipse CE95). **Không có CMakeLists, Gradle, JNI, EGL/GL context, draw loop** — Android app hiện không thể build; đây là khối "renderer chờ tích hợp".

## 2. VẤN ĐỀ CẤU TRÚC NGHIÊM TRỌNG NHẤT

- **[HIGH] Dead code + layering violation**: 2 file .cpp được biên dịch **trực tiếp vào `survival_core`** (`core/CMakeLists.txt:30-31`, include dir PUBLIC :34-38) và không được tham chiếu bởi bất kỳ render pipeline nào. Hệ quả: (a) GUI code nướng vào thư viện điều hướng (vi phạm phân tầng theo tinh thần Invariant 3 dù không dùng OS header); (b) không ship như một renderer mobile thật. **Khuyến nghị số 1 của thư mục này**: tách thành target riêng `survival_gui_renderer` + dựng khung Android (CMake toolchain + JNI + Gradle).

## 3. VẤN ĐỀ THEO FILE

### fps_governor.cpp
- ✅ Đơn vị đúng toàn bộ (nanoseconds; 16666667 ns = 60 FPS).
- **[LOW] Phase drift** (:33-36): reset `last_render_time_ns_ = now` thay vì `+= frame_interval_ns_` → trên màn 90 Hz target 60 FPS, frame time quantize theo callback (jitter 11/22 ms luân phiên). Fix chuẩn: tích lũy interval + catch-up clamp.
- **[INFO] Thiết kế cần reconsider**: pin ≤ 5% ép 0.1 FPS **kể cả khi đang di chuyển** — bản đồ đóng băng 10 s/lần trong tình huống sinh tồn đang đi bộ. Header :9 ghi kMovingOrRotating "30–60 FPS" nhưng code hard-fix 60 (:21).

### ribbon_renderer.cpp
- **[MEDIUM] Tràn uint16 index** (`ribbon_renderer.hpp:20`): vertex = 2N waypoint không có cap; N > 32767 (dễ vượt trên hành trình dài, 1 waypoint/bước) → `static_cast<uint16_t>(2i)` (:107-110) wrap → mesh hỏng/GL error. Fix: dùng `uint32_t` index hoặc cap N.
- **[MEDIUM] Float32 tọa độ tuyệt đối** (`ribbon_renderer.hpp:10-15`): ở tọa độ UTM (~500.000 m), spacing float32 ~0.03–0.06 m — rung/fold trên ribbon rộng 1–4 m. Fix: rebase về local origin (trừ tâm cụm) trước khi đổ vertex; không có code rebase nào tồn tại.
- **[LOW] Normal suy biến** (:74-92): waypoint trùng nhau → `len ≈ 0`, guard `> 1e-6` giữ normal (0,0) → tam giác zero-area. Cũng **không có miter join** — normal per-segment → nứt góc ở khúc cua gắt.
- ✅ Fan ellipse closure + winding đúng (:48-55, CCW nhất quán strip/fan).

## 4. TEST

`test_ribbon_mesh.cpp` + `test_native_gui_math.cpp` chỉ assert **số lượng** (2N vertex, 6(N−1) index, idx < 2N — điều kiện hiển nhiên) — không kiểm tra một vị trí vertex nào cả; sign flip/normal sai đều pass. Khuyến nghị: assert vị trí biên ribbon (±2 m từ path), hướng normal, và test overflow cap.

## 5. KHUYẾN NGHỊ LỘ TRÌNH TÍCH HỢP (theo đúng roadmap v2.4)

1. Tách target `survival_gui_renderer` khỏi survival_core (chỉ include `data_contracts.hpp`).
2. Thêm JNI bridge + EGL context + draw loop + Gradle wrapper.
3. Fix 3 bug renderer trên (uint16, float32 rebase, miter/normal) **trước** khi render dữ liệu thật.
4. FpsGovernor: interval accumulation; reconsider chính sách 0.1 FPS khi đang di chuyển.

**Điểm đánh giá chung: 4/10** — code gốc sạch về đơn vị nhưng dead code, chưa thể build app, 3 bug renderer sẽ nổ với dữ liệu dài/thật, test bề mặt.
