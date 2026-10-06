// ============================================================================
// [W1] WEB SERVER — dashboard web nhúng (cpp-httplib, chỉ đọc, an toàn LAN)
// ============================================================================
// Snapshot an toàn luồng: main loop (ImGui) đẩy WebState mỗi frame dưới mutex;
// web thread phục vụ JSON/SSE từ snapshot — không bao giờ chùng vòng lặp UI.
// Mặc định bind 127.0.0.1; mở LAN là opt-in qua UI. KHÔNG có endpoint ghi.
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace desktop {

// ---- Snapshot dữ liệu web (POD-ish, copy rẻ trừ vectors) ----
struct WebParticle {
    float east_m, north_m;
    bool alive;
};

struct WebCluster {
    double center_east_m, center_north_m;
    double semi_major_m, semi_minor_m, heading_rad;
};

struct WebSeries {
    std::vector<float> gns;       // cũ -> mới
    std::vector<float> speed;
    std::vector<float> vertical;
    std::vector<float> activity_conf;
};

struct WebState {
    // Telemetry
    double gns{0.0};
    double ce95_m{0.0};
    double est_east_m{0.0}, est_north_m{0.0};
    double true_east_m{0.0}, true_north_m{0.0};
    double true_heading_rad{0.0};
    double altitude_m{0.0};
    size_t total_steps{0};
    double total_distance_m{0.0};
    double battery{100.0};
    float fps{0.0F};

    // Hoạt động + thang
    int activity{0};              // MotionActivity enum
    double activity_conf{0.0};
    uint32_t stair_steps{0};
    double stair_riser_m{0.0};
    double stair_vertical_m{0.0};
    uint32_t stair_landings{0};

    double calibrated_containment{1.0};

    // An toàn (MissionState 0-3) + flags
    int safety_state{0};
    bool flag_imu{true}, flag_baro{true}, flag_mag_anomaly{false};
    bool flag_celestial{false}, flag_panic{false}, flag_immobile{false};
    bool flag_distress{false}, flag_diverged{false}, flag_stale{false}, flag_zupt{false};

    // Bản đồ
    std::vector<WebParticle> particles;
    std::vector<std::pair<double, double>> trail;
    std::vector<WebCluster> clusters;

    // Cứu hộ
    std::string packet_hex;
    std::string packet_sms;

    // Series
    WebSeries series;

    uint64_t timestamp_ms{0};
};

// JSON writer thuần (unit test được — không phụ thuộc httplib)
std::string buildStateJson(const WebState& s);
std::string jsonEscape(const std::string& in);

class WebServer {
public:
    WebServer() = default;
    ~WebServer() { stop(); }

    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    // web_dir: thư mục chứa index.html/app.js/style.css
    bool start(int port, bool bind_lan, const std::string& web_dir);
    void stop();

    [[nodiscard]] bool isRunning() const noexcept { return running_.load(); }
    [[nodiscard]] int port() const noexcept { return port_.load(); }
    [[nodiscard]] int sseClientCount() const noexcept { return sse_clients_.load(); }

    /// Main loop gọi mỗi frame: cập nhật snapshot (mutex bảo vệ)
    void publish(const WebState& state);

private:
    std::atomic<bool> running_{false};
    std::atomic<int> port_{8080};
    std::atomic<int> sse_clients_{0};
    std::thread server_thread_;
    std::thread sse_thread_;
    std::mutex state_mutex_;
    WebState snapshot_{};
    std::string web_dir_{"web"};
    void* svr_{nullptr};  // httplib::Server* (void để không leak header vào mọi TU)
};

} // namespace desktop
