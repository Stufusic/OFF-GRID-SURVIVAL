// ============================================================================
// UI PANELS — khai báo các hàm render panel ImGui (dockable dashboard)
// ============================================================================
#pragma once

#include "app_state.hpp"

namespace desktop {

// Tên cửa sổ: nhãn hiển thị song ngữ + ID ổn định sau "###" (dock layout dùng ID)
inline constexpr const char* kWinMap = "TAC MAP · BẢN ĐỒ###map";
inline constexpr const char* kWinReplay = "REPLAY · DỮ LIỆU THẬT###replay";
inline constexpr const char* kWinTelemetry = "TELEMETRY · ĐO ĐẠC###telemetry";
inline constexpr const char* kWinControls = "SIM CTRL · ĐIỀU KHIỂN###controls";
inline constexpr const char* kWinSafety = "SAFETY · AN TOÀN###safety";
inline constexpr const char* kWinRescue = "RESCUE PKT · CỨU HỘ###rescue";
inline constexpr const char* kWinWeb = "WEB LINK###web";

// Bản đồ tactical: grid UTM, contour DEM, PF cloud, 4 cụm giả thuyết + CE95,
// trail, vị trí thật/ước lượng. Zoom (scroll) + pan (kéo chuột) + fit-all.
void renderMapPanel(AppState& app, MapView& view);

// Telemetry: chỉ số lớn, ImPlot time-series, hoạt động, ma trận flags.
void renderTelemetryPanel(AppState& app);

// An toàn: 4 trạng thái mission + nút Cancel + bằng chứng.
void renderSafetyPanel(AppState& app);

// Gói cứu hộ 160-bit: hex + SMS + copy + lịch sử.
void renderRescuePanel(AppState& app);

// Điều khiển mô phỏng: nút/slider thay phím tắt.
void renderControlsPanel(AppState& app);

// Replay dữ liệu thật (U2): folder -> CsvLoader -> DataPlayer -> pipeline.
void renderReplayPanel(AppState& app);

// [W3] Web dashboard panel (start/stop server, URL, trạng thái)
class WebServer;
WebServer* getWebServer();
WebServer& g_web_server_ref();
bool& g_web_started_ref();
void renderWebPanel(AppState& app);

} // namespace desktop
