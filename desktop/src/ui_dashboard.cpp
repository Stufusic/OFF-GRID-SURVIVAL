// ============================================================================
// DASHBOARD PANELS — telemetry (ImPlot), safety, rescue, controls
// Phong cách "Tactical Instrument" (xem ui_theme.hpp): số đo mono lớn, nhãn
// viết hoa nhỏ song ngữ, ma trận ô trạng thái, thước phân đoạn.
// ============================================================================
#include "imgui.h"
#include "implot.h"
#include "ui_panels.hpp"
#include "ui_theme.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace desktop {

namespace {

using namespace theme;

const char* navModeLabel(NavigationMode mode) {
    switch (mode) {
        case NavigationMode::kGpsAided: return "GPS-AIDED";
        case NavigationMode::kPdrAided: return "PDR-AIDED";
        case NavigationMode::kTrnAided: return "TRN-AIDED";
        case NavigationMode::kDegraded: return "DEGRADED";
        case NavigationMode::kLost: return "LOST";
        default: return "KHAC";
    }
}

const char* positionSourceLabel(PositionSource src) {
    switch (src) {
        case PositionSource::kGps: return "GPS";
        case PositionSource::kPdr: return "PDR";
        case PositionSource::kTrn: return "TRN";
        case PositionSource::kGpsPdr: return "GPS+PDR";
        case PositionSource::kGpsTrn: return "GPS+TRN";
        case PositionSource::kPdrTrn: return "PDR+TRN";
        case PositionSource::kLastKnownGood: return "LAST KNOWN GOOD";
        default: return "KHAC";
    }
}

[[maybe_unused]] const char* powerModeLabel(PowerMode mode) {
    switch (mode) {
        case PowerMode::kBalanced: return "BALANCED";
        case PowerMode::kEnergyConservative: return "ENERGY CONSERVATIVE";
        case PowerMode::kCriticalSnapshot: return "CRITICAL SNAPSHOT";
        case PowerMode::kDisplayOnly: return "DISPLAY ONLY";
        default: return "KHAC";
    }
}

// Ngưỡng 3 mức: tốt (olive) / suy giảm (hổ phách) / nguy hiểm (đỏ)
ImU32 tri(float v, float good, float warn) {
    return v > good ? kOliveBright : (v > warn ? kAmber : kRed);
}

// "AA BB CC DD ..." -> "AABB CCDD ..." (nhóm 4 ký tự dễ đọc qua bộ đàm)
std::string groupHex4(const std::string& spaced) {
    std::string raw;
    for (char ch : spaced) {
        if (ch != ' ') raw += ch;
    }
    std::string out;
    for (size_t i = 0; i < raw.size(); ++i) {
        if (i > 0 && i % 4 == 0) out += ' ';
        out += raw[i];
    }
    return out;
}

// Nút kiểu "[ LABEL ]" — viền hổ phách khi hover
bool tacButton(const char* label, bool accent = false) {
    if (accent) {
        ImGui::PushStyleColor(ImGuiCol_Border, v4(kAmber));
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
    }
    const bool pressed = ImGui::Button(label);
    if (accent) ImGui::PopStyleColor(2);
    return pressed;
}

} // namespace

void renderTelemetryPanel(AppState& app) {
    if (!ImGui::Begin(kWinTelemetry)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    const auto& st = app.latest_state;
    const auto& c0 = st.clusters[0];
    char buf[64];

    // ---- Hàng số đo (3 cột, đường kẻ dọc mảnh giữa các ô) ----
    static float gns_anim = 0.0F;
    gns_anim += (static_cast<float>(st.global_navigation_score) - gns_anim) * 0.10F;
    if (ImGui::BeginTable("##readouts", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "%.0f%%", gns_anim * 100.0F);
        readout("GNS · TIN CẬY", buf, nullptr, tri(gns_anim, 0.70F, 0.40F));
        ImGui::SameLine(0.0F, 10.0F);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 18.0F);
        signalBars(gns_anim, tri(gns_anim, 0.70F, 0.40F), 18.0F);

        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "±%.1f", c0.ce95_semi_major_m);
        readout("CE95 · BÁN TRỤC", buf, "m", kAmber);

        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "%04zu", app.total_step_count);
        readout("STEPS · SỐ BƯỚC", buf, nullptr, kText);

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Dummy(ImVec2(0, 4));
        snprintf(buf, sizeof(buf), "%05.1f", app.total_distance_m);
        readout("DIST · QUÃNG ĐƯỜNG", buf, "m", kText);

        ImGui::TableNextColumn();
        ImGui::Dummy(ImVec2(0, 4));
        snprintf(buf, sizeof(buf), "%.1f", app.sim_altitude_m);
        readout("ALT · CAO ĐỘ BARO", buf, "m", kText);

        ImGui::TableNextColumn();
        ImGui::Dummy(ImVec2(0, 4));
        snprintf(buf, sizeof(buf), "%d%%", static_cast<int>(app.battery_soc));
        readout("PWR · PIN", buf, nullptr, tri(static_cast<float>(app.battery_soc), 40.0F, 15.0F));
        ImGui::EndTable();
    }
    ImGui::Dummy(ImVec2(0, 4));

    // ---- Hoạt động hiện tại + thước tin cậy 10 vạch ----
    sectionHeader("ACTIVITY", "HOẠT ĐỘNG");
    {
        const auto act = static_cast<uint8_t>(st.motion_activity);
        pushFont(fonts().mid);
        ImGui::PushStyleColor(ImGuiCol_Text, v4(activityColor(act) == kDim ? kText : activityColor(act)));
        ImGui::Text("%s", activityCode(act));
        ImGui::PopStyleColor();
        ImGui::SameLine(0.0F, 8.0F);
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kMuted));
        ImGui::Text("· %s", activityVi(act));
        ImGui::PopStyleColor();
        ImGui::PopFont();
        const float meter_w = std::min(220.0F, ImGui::GetContentRegionAvail().x * 0.40F);
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - meter_w - 46.0F);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 5.0F);
        segmentedMeter(st.activity_confidence, 10, kAmber, meter_w, 12.0F);
        ImGui::SameLine(0.0F, 8.0F);
        snprintf(buf, sizeof(buf), "%3.0f%%", st.activity_confidence * 100.0F);
        smallLabel(buf);
    }
    const auto& stair = app.pipeline.getActivityClassifier().getStairState();
    if (stair.step_count > 0) {
        snprintf(buf, sizeof(buf), "%u bậc · riser %.2fm · Δz %+.2fm · landing %u", stair.step_count,
                 stair.estimated_riser_m, stair.cumulative_vertical_m, stair.landing_count);
        smallLabel(buf, kMuted);
    }
    ImGui::Dummy(ImVec2(0, 2));

    // ---- Chuỗi tín hiệu (ring 60s) — kiểu máy hiện sóng ----
    sectionHeader("SIGNAL TRACE  60s @10Hz", "CHUỖI TÍN HIỆU");
    pushFont(fonts().label);
    if (ImPlot::BeginPlot("##series", ImVec2(-1, 190), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
        ImPlot::SetupAxes("t [s]", nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
        static float xs[600];
        static float ys_gns[600], ys_speed[600], ys_vert[600], ys_conf[600];

        const size_t n = app.series.gns.copyOut(ys_gns, 600);
        for (size_t i = 0; i < n; ++i) xs[i] = static_cast<float>(i) * 0.1F;

        ImPlot::SetNextLineStyle(v4(kAmber), 1.6F);
        ImPlot::PlotLine("GNS", xs, ys_gns, static_cast<int>(n));

        const size_t n_s = app.series.speed.copyOut(ys_speed, 600);
        ImPlot::SetNextLineStyle(v4(kText), 1.2F);
        ImPlot::PlotLine("V m/s", xs, ys_speed, static_cast<int>(n_s));

        const size_t n_v = app.series.vertical.copyOut(ys_vert, 600);
        ImPlot::SetNextLineStyle(v4(kOliveBright), 1.2F);
        ImPlot::PlotLine("Vz m/s", xs, ys_vert, static_cast<int>(n_v));

        const size_t n_c = app.series.activity_conf.copyOut(ys_conf, 600);
        ImPlot::SetNextLineStyle(v4(kBlueSoft), 1.2F);
        ImPlot::PlotLine("CONF", xs, ys_conf, static_cast<int>(n_c));

        ImPlot::EndPlot();
    }
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(0, 2));

    // ---- Ma trận sức khỏe 4×2 ----
    sectionHeader("SYS HEALTH", "SỨC KHỎE HỆ THỐNG");
    {
        const float gap = 6.0F;
        const float cw = (ImGui::GetContentRegionAvail().x - 3.0F * gap) / 4.0F;
        const ImVec2 cs(cw, 26.0F);
        struct C { const char* l; Cell s; };
        const C cells[8] = {
            {"IMU", st.flags.imu_healthy ? Cell::kOk : Cell::kFault},
            {"BARO", st.flags.baro_healthy ? Cell::kOk : Cell::kFault},
            {"MAG", !st.flags.mag_anomaly ? Cell::kOk : Cell::kFault},
            {"CEL", st.flags.celestial_available ? Cell::kInfoOn : Cell::kInfoOff},
            {"ZUPT", st.flags.zupt_active ? Cell::kInfoOn : Cell::kInfoOff},
            {"DIV", !st.flags.estimator_diverged ? Cell::kOk : Cell::kFault},
            {"STALE", !st.flags.estimator_stale ? Cell::kOk : Cell::kFault},
            {"IMMOB", !st.flags.immobility_warning ? Cell::kOk : Cell::kFault},
        };
        for (int i = 0; i < 8; ++i) {
            if (i % 4 != 0) ImGui::SameLine(0.0F, gap);
            statusCell(cells[i].l, cells[i].s, cs);
        }
    }
    ImGui::Dummy(ImVec2(0, 2));
    snprintf(buf, sizeof(buf), "%s · %s · age %.1fs", navModeLabel(st.nav_mode),
             positionSourceLabel(st.position_source), st.position_age_s);
    kvRow("NAV MODE", buf);
    snprintf(buf, sizeof(buf), "%.0f%%  ·  %u cụm", st.calibrated_containment_probability * 100.0,
             st.active_clusters_count);
    kvRow("P(TRUE ∈ CE95)", buf);

    ImGui::End();
}

void renderSafetyPanel(AppState& app) {
    if (!ImGui::Begin(kWinSafety)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    const auto state = app.safety_auditor.getState();
    const char* text = "STATUS: NOMINAL — AN TOÀN BÌNH THƯỜNG";
    ImU32 col = kOliveBright;
    if (state == SafetyState::kPreAlert) {
        text = "PRE-ALERT — PHÁT HIỆN NGÃ · ĐẾM NGƯỢC 45s";
        col = kAmber;
    } else if (state == SafetyState::kDistressLocked) {
        text = "DISTRESS — KHÓA CỨU NẠN SOS TỰ ĐỘNG";
        col = kRed;
    } else if (state == SafetyState::kPanicCirclingWarning) {
        text = "CIRCLING — ĐI VÒNG QUANH, MẤT PHƯƠNG HƯỚNG";
        col = kYellow;
    }
    statusStrip(text, col, state != SafetyState::kNormal, 32.0F);

    if (state != SafetyState::kNormal) {
        ImGui::Dummy(ImVec2(0, 2));
        if (tacButton("[ HỦY BÁO ĐỘNG — TÔI ỔN ]", true)) {
            app.safety_auditor.cancelAlert();
            app.latest_state.safety_state = app.safety_auditor.getState();
        }
    }

    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("SENSOR EVIDENCE", "BẰNG CHỨNG CẢM BIẾN");
    const auto& ev = app.pipeline.getActivityClassifier().getLastEstimate().evidence;
    char buf[96];
    snprintf(buf, sizeof(buf), "%.2f Hz", ev.cadence_hz);
    kvRow("CADENCE", buf);
    snprintf(buf, sizeof(buf), "%.2f  /  %.2f", ev.vertical_energy, ev.horizontal_energy);
    kvRow("ENERGY V / H", buf);
    snprintf(buf, sizeof(buf), "%.1f m/s²  /  %.1f", ev.impact_score, ev.freefall_score);
    kvRow("IMPACT / FREEFALL", buf);
    snprintf(buf, sizeof(buf), "%+.2f m/s", ev.baro_vertical_rate);
    kvRow("BARO RATE", buf);
    snprintf(buf, sizeof(buf), "%.2f m/s", ev.gnss_speed);
    kvRow("GNSS SPEED", buf);

    ImGui::End();
}

void renderRescuePanel(AppState& app) {
    if (!ImGui::Begin(kWinRescue)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    const auto& c0 = app.latest_state.clusters[0];
    const auto [lat, lon] = app.geo_origin.enToLatLon(c0.center_easting_m, c0.center_northing_m);
    const auto raw = Packet160Serializer::encode(
        static_cast<uint32_t>(app.current_time_ns / 1000000000ULL),
        lat, lon, app.sim_altitude_m,
        c0.ce95_semi_major_m * 0.5, c0.ce95_semi_minor_m * 0.5,
        c0.ellipse_heading_rad * 180.0 / 3.14159265358979323846,
        0x01, static_cast<uint8_t>(app.battery_soc), 1);
    OutboundRescuePacket160 out{};
    out.data = raw;
    const auto sms = SosEgressManager::buildCompactSmsPayload(out, app.latest_state, app.battery_soc);
    std::string sms_str(sms.begin(), sms.end());

    sectionHeader("POSITION FIX", "VỊ TRÍ ƯỚC LƯỢNG");
    char buf[96];
    snprintf(buf, sizeof(buf), "%+.5f°", lat);
    readout("LAT", buf, nullptr, kAmber);
    ImGui::SameLine(0.0F, 36.0F);
    snprintf(buf, sizeof(buf), "%+.5f°", lon);
    readout("LON", buf, nullptr, kAmber);

    smallLabel("GEO ORIGIN · GỐC TỌA ĐỘ");
    ImGui::PushItemWidth(150.0F);
    ImGui::InputDouble("LAT₀", &app.geo_origin.base_lat_deg, 0.001, 0.01, "%.5f");
    ImGui::SameLine();
    ImGui::InputDouble("LON₀", &app.geo_origin.base_lon_deg, 0.001, 0.01, "%.5f");
    ImGui::PopItemWidth();

    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("PACKET 160-BIT", "GÓI TIN 20 BYTE");
    pushFont(fonts().mid);
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
    ImGui::TextWrapped("%s", groupHex4(toHexDump(raw.data(), raw.size())).c_str());
    ImGui::PopStyleColor();
    ImGui::PopFont();

    smallLabel("SMS PAYLOAD");
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kText));
    ImGui::TextWrapped("%s", sms_str.c_str());
    ImGui::PopStyleColor();

    ImGui::Dummy(ImVec2(0, 2));
    if (tacButton("[ COPY SMS ]", true)) {
        ImGui::SetClipboardText(sms_str.c_str());
    }
    ImGui::SameLine();
    if (tacButton("[ LƯU LỊCH SỬ ]")) {
        app.updateRescuePacket();
    }

    if (!app.rescue_history.empty()) {
        ImGui::Dummy(ImVec2(0, 4));
        snprintf(buf, sizeof(buf), "LOG · %zu GÓI", app.rescue_history.size());
        sectionHeader(buf);
        ImGui::BeginChild("rescue_history", ImVec2(0, 130), ImGuiChildFlags_Border);
        int idx = 0;
        for (auto it = app.rescue_history.rbegin(); it != app.rescue_history.rend(); ++it) {
            ImGui::Text("#%02d  %+.5f, %+.5f   CE95 %3.0fm   PWR %d%%", idx++, it->lat_deg, it->lon_deg,
                        it->ce95_m, it->battery);
            smallLabel(groupHex4(it->hex).c_str(), kDim);
        }
        ImGui::EndChild();
    }

    ImGui::End();
}

void renderControlsPanel(AppState& app) {
    if (!ImGui::Begin(kWinControls)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    ImGuiIO& io = ImGui::GetIO();

    sectionHeader("AUTOPILOT", "TỰ ĐỘNG ĐI");
    if (tacButton(app.auto_walk ? "[ ■ DỪNG ]" : "[ ▶ CHẠY ]", true)) {
        app.auto_walk = !app.auto_walk;
    }
    ImGui::SameLine();
    smallLabel(io.WantCaptureKeyboard ? "KEYBOARD → UI" : "KEYBOARD READY", kMuted);
    {
        float interval_f = static_cast<float>(app.step_interval_s);
        ImGui::SetNextItemWidth(-140.0F);
        if (ImGui::SliderFloat("CHU KỲ BƯỚC", &interval_f, 0.30F, 1.20F, "%.2f s",
                               ImGuiSliderFlags_AlwaysClamp)) {
            app.step_interval_s = interval_f;
        }
    }

    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("MANUAL STEP", "BƯỚC TAY");
    if (tacButton("[ W ↑ TIẾN ]")) app.simulateStep(0.75, 0.0);
    ImGui::SameLine();
    if (tacButton("[ A ← TRÁI ]")) app.simulateStep(0.75, -0.2618);
    ImGui::SameLine();
    if (tacButton("[ D → PHẢI ]")) app.simulateStep(0.75, 0.2618);

    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("FAULT INJECTION", "TIÊM SỰ KIỆN");
    ImGui::PushStyleColor(ImGuiCol_Border, v4(kRed));
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kRed));
    if (ImGui::Button("[ F  MÔ PHỎNG CÚ NGÃ ]")) app.triggerFallEvent();
    ImGui::PopStyleColor(2);
    {
        float battery_f = static_cast<float>(app.battery_soc);
        ImGui::SetNextItemWidth(-140.0F);
        if (ImGui::SliderFloat("PIN", &battery_f, 5.0F, 100.0F, "%.0f%%", ImGuiSliderFlags_AlwaysClamp)) {
            app.battery_soc = battery_f;
            app.resource_governor.updateBatterySoc(app.battery_soc);
        }
    }
    ImGui::Checkbox("GPS HANDOFF ĐỊNH KỲ (G)", &app.gps_enabled);
    if (tacButton("[ R  RESET MÔ PHỎNG ]")) {
        app.initialize();
    }

    ImGui::Dummy(ImVec2(0, 6));
    sectionHeader("KEYMAP", "PHÍM TẮT");
    smallLabel("W/A/D  bước     SPACE  auto     F  ngã     C  hủy báo động", kMuted);
    smallLabel("G  GPS     B  xả pin     R  reset", kMuted);
    char buf[64];
    snprintf(buf, sizeof(buf), "FPS %.1f · governor theo pin", io.Framerate);
    smallLabel(buf, kDim);

    ImGui::End();
}

} // namespace desktop
