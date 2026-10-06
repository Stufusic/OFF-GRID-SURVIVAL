// ============================================================================
// [W3] WEB DASHBOARD PANEL — Start/Stop, port, toggle LAN, trạng thái
// ============================================================================
#include "imgui.h"
#include "ui_panels.hpp"
#include "ui_theme.hpp"
#include "web_server.hpp"

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace desktop {

namespace {
WebServer g_web_server;
char g_port_buf[8] = "8080";
bool g_bind_lan = false;
bool g_web_started = false;

std::string localURL(int port) {
    return "http://127.0.0.1:" + std::to_string(port) + "/";
}
} // namespace

WebServer* getWebServer() {
    return &g_web_server;
}

bool& g_web_started_ref() {
    return g_web_started;
}

WebServer& g_web_server_ref() {
    return g_web_server;
}

void publishWebSnapshot(AppState& app);

void renderWebPanel(AppState& app) {
    using namespace theme;
    // Snapshot phải được publish mỗi frame kể cả khi tab đang ẩn (panel dock dạng tab)
    if (g_web_server.isRunning()) publishWebSnapshot(app);

    if (!ImGui::Begin(kWinWeb)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    sectionHeader("WEB LINK", "DASHBOARD TRÌNH DUYỆT");
    smallLabel("Chỉ-đọc. Xem trên máy này hoặc điện thoại cùng LAN. Không có endpoint ghi/lệnh.", kMuted);
    ImGui::Dummy(ImVec2(0, 4));

    ImGui::SetNextItemWidth(90.0F);
    ImGui::InputText("PORT", g_port_buf, sizeof(g_port_buf),
                     ImGuiInputTextFlags_CharsDecimal);
    int port = atoi(g_port_buf);
    if (port < 1024 || port > 65535) port = 8080;
    ImGui::SameLine(0.0F, 20.0F);
    ImGui::Checkbox("MỞ LAN (0.0.0.0)", &g_bind_lan);
    if (g_bind_lan) {
        smallLabel("▲ CẢNH BÁO: mọi thiết bị trong mạng đều xem được.", kAmber);
    }

    ImGui::PushStyleColor(ImGuiCol_Border, v4(kAmber));
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
    if (!g_web_started) {
        if (ImGui::Button("[ ▶ START SERVER ]")) {
            const std::string web_dir = (std::filesystem::path("../web")).string();
            g_web_started = g_web_server.start(port, g_bind_lan, web_dir);
        }
    } else {
        if (ImGui::Button("[ ■ STOP SERVER ]")) {
            g_web_server.stop();
            g_web_started = false;
        }
    }
    ImGui::PopStyleColor(2);

    ImGui::SameLine(0.0F, 14.0F);
    if (g_web_server.isRunning()) {
        char st[48];
        snprintf(st, sizeof(st), "● ONLINE :%d", g_web_server.port());
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kOliveBright));
        ImGui::TextUnformatted(st);
        ImGui::PopStyleColor();
    } else {
        ImGui::TextDisabled("○ OFFLINE");
    }

    if (g_web_server.isRunning()) {
        ImGui::Dummy(ImVec2(0, 4));
        const std::string url = localURL(g_web_server.port());
#ifdef _WIN32
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
        if (ImGui::SmallButton(url.c_str())) {
            // Mở trình duyệt mặc định
            const std::string cmd = "start " + url;
            (void)system(cmd.c_str());
        }
        ImGui::PopStyleColor();
#else
        ImGui::TextUnformatted(url.c_str());
#endif
        char buf[48];
        snprintf(buf, sizeof(buf), "%d", g_web_server.sseClientCount());
        kvRow("SSE CLIENTS", buf);
    }

    ImGui::End();
}

void publishWebSnapshot(AppState& app) {
    WebState ws{};
    {
        {
        ws.gns = app.latest_state.global_navigation_score;
        const auto& c0 = app.latest_state.clusters[0];
        ws.ce95_m = c0.ce95_semi_major_m;
        ws.est_east_m = c0.center_easting_m;
        ws.est_north_m = c0.center_northing_m;
        ws.true_east_m = app.true_easting_m;
        ws.true_north_m = app.true_northing_m;
        ws.true_heading_rad = app.true_heading_rad;
        ws.altitude_m = app.sim_altitude_m;
        ws.total_steps = app.total_step_count;
        ws.total_distance_m = app.total_distance_m;
        ws.battery = app.battery_soc;
        ws.fps = app.fps_measured;
        ws.activity = static_cast<int>(app.latest_state.motion_activity);
        ws.activity_conf = app.latest_state.activity_confidence;
        const auto& stair = app.pipeline.getActivityClassifier().getStairState();
        ws.stair_steps = stair.step_count;
        ws.stair_riser_m = stair.estimated_riser_m;
        ws.stair_vertical_m = stair.cumulative_vertical_m;
        ws.stair_landings = stair.landing_count;
        ws.safety_state = static_cast<int>(app.safety_auditor.getState());
        const auto& fl = app.latest_state.flags;
        ws.flag_imu = fl.imu_healthy;
        ws.flag_baro = fl.baro_healthy;
        ws.flag_mag_anomaly = fl.mag_anomaly;
        ws.flag_celestial = fl.celestial_available;
        ws.flag_panic = fl.panic_circling;
        ws.flag_immobile = fl.immobility_warning;
        ws.flag_distress = fl.distress_sos_active;
        ws.flag_diverged = fl.estimator_diverged;
        ws.flag_stale = fl.estimator_stale;
        ws.flag_zupt = fl.zupt_active;
        ws.calibrated_containment = app.latest_state.calibrated_containment_probability;

        for (const auto& p : app.pipeline.getParticleFilter().getParticles()) {
            ws.particles.push_back({p.easting_m, p.northing_m, p.is_alive});
        }
        ws.trail = app.trail_history;
        for (int ci = 0; ci < app.latest_state.active_clusters_count && ci < 4; ++ci) {
            const auto& c = app.latest_state.clusters[ci];
            ws.clusters.push_back({c.center_easting_m, c.center_northing_m,
                                   c.ce95_semi_major_m, c.ce95_semi_minor_m,
                                   c.ellipse_heading_rad});
        }
        if (!app.rescue_history.empty()) {
            const auto& last = app.rescue_history.back();
            ws.packet_hex = last.hex;
            ws.packet_sms = last.sms;
        }
        {
            std::vector<float> tmp(600);
            size_t n = app.series.gns.copyOut(tmp.data(), 600);
            ws.series.gns.assign(tmp.begin(), tmp.begin() + static_cast<long>(n));
            n = app.series.speed.copyOut(tmp.data(), 600);
            ws.series.speed.assign(tmp.begin(), tmp.begin() + static_cast<long>(n));
            n = app.series.vertical.copyOut(tmp.data(), 600);
            ws.series.vertical.assign(tmp.begin(), tmp.begin() + static_cast<long>(n));
            n = app.series.activity_conf.copyOut(tmp.data(), 600);
            ws.series.activity_conf.assign(tmp.begin(), tmp.begin() + static_cast<long>(n));
        }
        ws.timestamp_ms = app.current_time_ns / 1000000ULL;

        g_web_server.publish(ws);
        }
    }
}

} // namespace desktop
