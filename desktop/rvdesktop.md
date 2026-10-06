# CODE REVIEW — THƯ MỤC `desktop/` (CẬP NHẬT 2026-09-28)

## Kiến trúc MỚI (Dear ImGui + Win32/DX11) — thay thế monolith GDI+ cũ

| File | Vai trò |
|---|---|
| `src/app_main.cpp` | Win32 + DX11 bootstrap, vòng lặp, theme AMOLED, font Segoe UI (glyph tiếng Việt), dockspace, phím tắt |
| `src/app_state.hpp` | Trạng thái mô phỏng + wiring engine (pipeline/safety/governor), sim waveform 100Hz "đẩy sớm" |
| `src/hud_math.hpp` | Toán thuần test được: MapView zoom/pan/fit, ellipse CE95, RingBuffer, GeoOrigin, hex dump |
| `src/ui_map.cpp` | Bản đồ tactical: grid UTM, contour, PF cloud (culling), 4 cụm giả thuyết + CE95 riêng, trail, zoom scroll/pan chuột |
| `src/ui_dashboard.cpp` | Telemetry (chỉ số lớn + ImPlot 4 series @10Hz + stair state + ma trận flags), Safety, Rescue (copy/clipboard + lịch sử), Controls |
| `src/ui_replay.cpp` | U2: phát lại dataset thật (CsvLoader → DataPlayer → pipeline), play/pause/speed 0.5–8×, timeline hoạt động tô màu |
| `src/ui_web.cpp` + `src/web_server.{hpp,cpp}` | [W] Web dashboard nhúng: cpp-httplib (FetchContent), SSE 4Hz + polling fallback, snapshot mutex, chỉ-đọc, bind 127.0.0.1 mặc định (LAN opt-in) |
| `web/` | index.html + app.js + style.css — dashboard web thuần JS (map canvas cùng quy ước MapView, series chart, cards, safety banner, rescue) |

**Đã sửa so với review cũ (score 6/10):**
1. ✅ [MEDIUM] Gyro sai đơn vị — sim giờ bơm rate (rad/s) qua 55 mẫu 100Hz/bước (kèm dạng sóng bước thật → StepDetector/classifier hoạt động thật).
2. ✅ Phím 'G' chết → GPS handoff định kỳ qua `updateAnchorPosition()`.
3. ✅ Nhãn FPS hardcode → FPS thật từ vòng lặp.
4. ✅ Coords Hà Nội hardcode → `GeoOrigin` chỉnh được trong UI.
5. ✅ Thiếu WIN32_EXECUTABLE → đã set (hết console rãnh).
6. ✅ SMS UTF-8 mojibake → ImGui UTF-8 thuần + font Việt.
7. ✅ Map size âm/clipping → canvas ImGui tự co giãn + culling viewport.
8. ✅ Zero desktop tests → `tests/test_desktop_hud.cpp` (8 test, 191/191 toàn dự án).

**Điểm đánh giá mới: 8/10** — còn lại: `imu_health_` write-only (thuộc core), chưa có golden screenshot test, text chưa dùng đầy đủ dấu tiếng Việt (font đã hỗ trợ).

## WEB DASHBOARD [W] — đã kiểm chứng 2026-10-03
- Endpoints: `/` `/app.js` `/style.css` (static), `/api/health`, `/api/state` (JSON 1 lần), `/api/events` (SSE ~4Hz).
- Snapshot mutex: main loop publish mỗi frame, web thread đọc — không chùng UI.
- Smoke đã pass: health OK, state JSON đầy đủ, static 200, **SSE 8 events/2s**; browser test xác nhận SSE badge "trực tiếp", cards/canvas/series render với dữ liệu thật.
- JSON writer thuần `buildStateJson`/`jsonEscape` — 5 unit test (`test_web_json.cpp`).
- Bảo mật: chỉ-đọc, bind local mặc định, LAN là opt-in có cảnh báo trong UI.
- Hạn chế đã biết: auto-fit ban đầu có thể "trôi" khi trail dài (bấm Fit all); SSE chiếm 1 thread httplib/client (pool mặc định 8).

## Lịch sử cũ
Monolith Win32/GDI+ `main_desktop_gui.cpp` (517 dòng) đã bị THAY THẾ và XÓA —
review dưới đây giữ lại để tham khảo các phát hiện gốc.

---
