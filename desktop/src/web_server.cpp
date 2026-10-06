// ============================================================================
// [W1] WEB SERVER implementation — cpp-httplib + SSE
// ============================================================================
#include "web_server.hpp"

#include <httplib.h>

#include <algorithm>
#include <filesystem>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace desktop {

// ----------------------------------------------------------------------------
// JSON writer thuần
// ----------------------------------------------------------------------------
std::string jsonEscape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += c;
            }
        }
    }
    return out;
}

namespace {
void appendSeries(std::ostringstream& json, const char* name, const std::vector<float>& v) {
    json << ",\"" << name << "\":[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) json << ',';
        json << static_cast<double>(v[i]);
    }
    json << ']';
}
} // namespace

std::string buildStateJson(const WebState& s) {
    std::ostringstream json;
    json << std::fixed;
    json.precision(3);

    json << "{\"gns\":" << s.gns
         << ",\"ce95_m\":" << s.ce95_m
         << ",\"est\":{\"e\":" << s.est_east_m << ",\"n\":" << s.est_north_m << "}"
         << ",\"true\":{\"e\":" << s.true_east_m << ",\"n\":" << s.true_north_m
         << ",\"heading\":" << s.true_heading_rad << "}"
         << ",\"altitude\":" << s.altitude_m
         << ",\"steps\":" << s.total_steps
         << ",\"distance\":" << s.total_distance_m
         << ",\"battery\":" << s.battery
         << ",\"fps\":" << s.fps
         << ",\"activity\":" << s.activity
         << ",\"activity_conf\":" << s.activity_conf
         << ",\"stair\":{\"steps\":" << s.stair_steps
         << ",\"riser\":" << s.stair_riser_m
         << ",\"vertical\":" << s.stair_vertical_m
         << ",\"landings\":" << s.stair_landings << "}"
         << ",\"calibrated_containment\":" << s.calibrated_containment
         << ",\"safety_state\":" << s.safety_state
         << ",\"flags\":{\"imu\":" << (s.flag_imu ? "true" : "false")
         << ",\"baro\":" << (s.flag_baro ? "true" : "false")
         << ",\"mag_anomaly\":" << (s.flag_mag_anomaly ? "true" : "false")
         << ",\"celestial\":" << (s.flag_celestial ? "true" : "false")
         << ",\"panic\":" << (s.flag_panic ? "true" : "false")
         << ",\"immobile\":" << (s.flag_immobile ? "true" : "false")
         << ",\"distress\":" << (s.flag_distress ? "true" : "false")
         << ",\"diverged\":" << (s.flag_diverged ? "true" : "false")
         << ",\"stale\":" << (s.flag_stale ? "true" : "false")
         << ",\"zupt\":" << (s.flag_zupt ? "true" : "false") << "}"
         << ",\"packet_hex\":\"" << jsonEscape(s.packet_hex) << "\""
         << ",\"packet_sms\":\"" << jsonEscape(s.packet_sms) << "\""
         << ",\"timestamp_ms\":" << s.timestamp_ms;

    // Particles (chỉ hạt sống để giảm kích thước)
    json << ",\"particles\":[";
    bool first = true;
    for (const auto& p : s.particles) {
        if (!p.alive) continue;
        if (!first) json << ',';
        json << '[' << static_cast<double>(p.east_m) << ',' << static_cast<double>(p.north_m) << ']';
        first = false;
    }
    json << ']';

    // Trail
    json << ",\"trail\":[";
    for (size_t i = 0; i < s.trail.size(); ++i) {
        if (i) json << ',';
        json << '[' << s.trail[i].first << ',' << s.trail[i].second << ']';
    }
    json << ']';

    // Clusters
    json << ",\"clusters\":[";
    for (size_t i = 0; i < s.clusters.size(); ++i) {
        if (i) json << ',';
        const auto& c = s.clusters[i];
        json << "{\"e\":" << c.center_east_m << ",\"n\":" << c.center_north_m
             << ",\"a\":" << c.semi_major_m << ",\"b\":" << c.semi_minor_m
             << ",\"h\":" << c.heading_rad << "}";
    }
    json << ']';

    appendSeries(json, "series_gns", s.series.gns);
    appendSeries(json, "series_speed", s.series.speed);
    appendSeries(json, "series_vertical", s.series.vertical);
    appendSeries(json, "series_conf", s.series.activity_conf);

    json << "}";
    return json.str();
}

// ----------------------------------------------------------------------------
// WebServer
// ----------------------------------------------------------------------------
bool WebServer::start(int port, bool bind_lan, const std::string& web_dir) {
    if (running_.load()) {
        stop();
    }
    // [W4] Giải thư mục web theo nhiều ứng viên (CWD khác nhau: build/desktop,
    // build/, hoặc gốc dự án khi smoke test)
    namespace fs = std::filesystem;
    web_dir_ = web_dir;
    const fs::path candidates[] = {fs::path(web_dir), fs::path("../web"),
                                   fs::path("../../desktop/web"),
                                   fs::path("../../../desktop/web")};
    for (const auto& cand : candidates) {
        if (fs::exists(cand / "index.html")) {
            web_dir_ = cand.string();
            break;
        }
    }
    if (!fs::exists(fs::path(web_dir_) / "index.html")) {
        return false;  // không tìm thấy UI
    }
    port_.store(port);

    auto* svr = new httplib::Server();
    svr_ = svr;

    // Static files
    svr->set_mount_point("/", web_dir_);

    // Health check
    svr->Get("/api/health", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // Snapshot JSON 1 lần (fallback polling)
    svr->Get("/api/state", [this](const httplib::Request&, httplib::Response& res) {
        std::string json;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            json = buildStateJson(snapshot_);
        }
        res.set_content(json, "application/json");
    });

    // SSE: push snapshot 4Hz
    svr->Get("/api/events", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-cache");
        res.set_chunked_content_provider(
            "text/event-stream",
            [this](size_t /*offset*/, httplib::DataSink& sink) -> bool {
                sse_clients_.fetch_add(1);
                std::string last_json;
                // Vòng push 4Hz cho đến khi client ngắt
                while (running_.load() && sink.is_writable()) {
                    std::string json;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        json = buildStateJson(snapshot_);
                    }
                    if (json != last_json) {
                        const std::string payload = "data: " + json + "\n\n";
                        if (!sink.write(payload.data(), payload.size())) {
                            break;
                        }
                        last_json = std::move(json);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }
                sse_clients_.fetch_sub(1);
                return false;  // kết thúc chunked stream
            });
    });

    // Chạy server trên thread riêng
    const char* host = bind_lan ? "0.0.0.0" : "127.0.0.1";
    server_thread_ = std::thread([svr, host, port]() {
        svr->listen(host, port);
    });
    // wait_until_ready() trong v0.15.3 không có timeout — poll is_running()
    int waited_ms = 0;
    while (!svr->is_running() && waited_ms < 3000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        waited_ms += 25;
    }
    if (!svr->is_running()) {
        stop();
        return false;
    }

    running_.store(true);

    // Thread SSE client-count driver (push snapshot cho các connection mới)
    // — SSE thực tế chạy trong provider của từng connection (ở trên).
    return true;
}

void WebServer::stop() {
    if (!running_.load() && svr_ == nullptr) return;
    running_.store(false);
    if (svr_ != nullptr) {
        static_cast<httplib::Server*>(svr_)->stop();
    }
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    if (svr_ != nullptr) {
        delete static_cast<httplib::Server*>(svr_);
        svr_ = nullptr;
    }
    sse_clients_.store(0);
}

void WebServer::publish(const WebState& state) {
    if (!running_.load()) return;
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot_ = state;
}

} // namespace desktop
