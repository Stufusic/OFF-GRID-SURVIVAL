// ============================================================================
// APP STATE — trạng thái mô phỏng + wiring engine (dùng chung các panel)
// ============================================================================
#pragma once

#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/safety/safety_auditor.hpp"
#include "survival_core/safety/resource_governor.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include "survival_core/comms/sos_egress_manager.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_harness/csv_loader.hpp"
#include "survival_harness/data_player.hpp"
#include "hud_math.hpp"

#include <deque>
#include <string>
#include <vector>

namespace desktop {

// Wiring engine: các kiểu engine dùng trực tiếp trong trạng thái app
using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;
using namespace survival::safety;
using namespace survival::comms;
using namespace survival::harness;

// Cấu hình chỉ số series cho ImPlot
struct TelemetrySeries {
    RingBuffer<float, 600> gns{};
    RingBuffer<float, 600> speed{};
    RingBuffer<float, 600> vertical{};
    RingBuffer<float, 600> activity_conf{};
    RingBuffer<float, 600> baro_alt{};
    double sample_dt_s{0.0};
    uint64_t last_sample_ns{0};

    void maybeSample(double value_gns, double v_speed, double v_vert,
                     double act_conf, double alt, uint64_t now_ns) {
        if (last_sample_ns != 0 && now_ns < last_sample_ns + 100000000ULL) {
            return;  // 10 Hz đủ cho plot 60s
        }
        sample_dt_s = (last_sample_ns == 0) ? 0.1
            : static_cast<double>(now_ns - last_sample_ns) * 1e-9;
        last_sample_ns = now_ns;
        gns.push(static_cast<float>(value_gns));
        speed.push(static_cast<float>(v_speed));
        vertical.push(static_cast<float>(v_vert));
        activity_conf.push(static_cast<float>(act_conf));
        baro_alt.push(static_cast<float>(alt));
    }
};

struct RescuePacketRecord {
    std::string hex;
    std::string sms;
    double lat_deg{0.0};
    double lon_deg{0.0};
    double ce95_m{0.0};
    int battery{0};
};

struct AppState {
    // Engine wiring (direct C++ — truy cập PF cloud + pipeline nội bộ)
    DemMap dem_map{};
    MultiModalFusionPipeline pipeline{};
    SafetyAuditor safety_auditor{};
    ResourceGovernor resource_governor{};

    // Vị trí thật mô phỏng
    double true_easting_m{1000.0};
    double true_northing_m{750.0};
    double true_heading_rad{0.0};
    double sim_altitude_m{1500.0};

    // Lịch sử + thống kê
    std::vector<std::pair<double, double>> trail_history{};
    size_t total_step_count{0};
    double total_distance_m{0.0};

    // Điều khiển mô phỏng
    bool auto_walk{true};
    bool gps_enabled{false};
    double battery_soc{84.0};
    double step_interval_s{0.55};      // 1.8 Hz
    double sim_wall_accum{0.0};        // tích lũy wall-clock cho pacing bước
    uint64_t current_time_ns{1700000000000000000ULL};

    // Geo origin cho gói cứu hộ (chỉnh được — bỏ hardcode Hà Nội)
    GeoOrigin geo_origin{};

    // Trạng thái ước lượng mới nhất
    NavigationUncertaintyState latest_state{};

    // Series + rescue history
    TelemetrySeries series{};
    std::deque<RescuePacketRecord> rescue_history;

    // FPS thật
    float fps_measured{0.0F};

    // Sub-frame waveform (1 bước = nhiều mẫu 100Hz để StepDetector hoạt động thật)
    static constexpr size_t kSamplesPerStep = 55;  // 0.55s @ 100Hz

    void initialize() {
        dem_map.createSyntheticCone(2000.0, 2000.0, 10.0, 1000.0, 1000.0, 2000.0, 1000.0, 500.0);
        true_easting_m = 1000.0;
        true_northing_m = 750.0;
        true_heading_rad = 0.0;
        const auto elev = dem_map.getElevationBilinear(true_easting_m, true_northing_m);
        sim_altitude_m = elev.value_or(1500.0);
        pipeline.initialize(EastingNorthingAlt{true_easting_m, true_northing_m, sim_altitude_m}, dem_map);
        trail_history.clear();
        trail_history.emplace_back(true_easting_m, true_northing_m);
        total_step_count = 0;
        total_distance_m = 0.0;
        series = TelemetrySeries{};
        rescue_history.clear();
        sim_wall_accum = 0.0;
    }

    // 1 bước mô phỏng = 55 mẫu 100Hz dạng sóng "đẩy sớm" (khớp cực tính đã audit)
    void simulateStep(double stride_len_m, double turn_delta_rad) {
        constexpr double kSubDt = 0.01;  // 100 Hz
        true_heading_rad = math::normalizeHeadingRad(true_heading_rad + turn_delta_rad);

        const double gyro_rate = turn_delta_rad / step_interval_s;  // [C-U1] FIX gyro unit bug

        for (size_t i = 0; i < kSamplesPerStep; ++i) {
            const double phase = static_cast<double>(i) / static_cast<double>(kSamplesPerStep);
            current_time_ns += static_cast<uint64_t>(kSubDt * 1e9);

            // Tịnh tiến vị trí thật dần theo phase
            const double stride_sub = stride_len_m / static_cast<double>(kSamplesPerStep);
            true_easting_m += stride_sub * std::sin(true_heading_rad);
            true_northing_m += stride_sub * std::cos(true_heading_rad);

            SensorFrame frame{};
            frame.timestamp_ns = current_time_ns;
            frame.has_imu = true;
            frame.has_baro = true;
            frame.has_mag = true;

            // Dạng sóng dọc "đẩy sớm" (đã audit: skewness dương = leo, v2h dải thang)
            const double a_dyn = (phase < 0.50)
                ? 4.5 * std::sin(phase / 0.50 * 3.14159265358979323846)
                : -1.2 * std::sin((phase - 0.50) / 0.50 * 3.14159265358979323846);
            frame.accel_m_s2 = {0.0F, 0.0F, static_cast<float>(9.81 + a_dyn)};
            frame.gyro_rad_s = {0.0F, 0.0F, static_cast<float>(gyro_rate)};

            const auto alt_opt = dem_map.getElevationBilinear(true_easting_m, true_northing_m);
            sim_altitude_m = alt_opt.value_or(sim_altitude_m);
            const double p_hpa = 1013.25 * std::pow(1.0 - 2.25577e-5 * sim_altitude_m, 5.25588);
            frame.pressure_hpa = static_cast<float>(p_hpa);

            frame.mag_uT = {
                static_cast<float>(20.0 * std::sin(true_heading_rad)),
                static_cast<float>(20.0 * std::cos(true_heading_rad)),
                -40.0F
            };

            if (gps_enabled && (total_step_count % 10 == 0) && i == 0) {
                // GPS handoff định kỳ: neo về vị trí thật (mô phỏng fix GPS)
                pipeline.updateAnchorPosition(
                    EastingNorthingAlt{true_easting_m, true_northing_m, sim_altitude_m}, 5.0);
            }

            safety_auditor.processMotionSample(
                Vector3f{frame.accel_m_s2[0], frame.accel_m_s2[1], frame.accel_m_s2[2]},
                Vector3f{frame.gyro_rad_s[0], frame.gyro_rad_s[1], frame.gyro_rad_s[2]},
                current_time_ns);

            const auto state_opt = pipeline.processSensorFrame(frame);
            if (state_opt.has_value()) {
                latest_state = *state_opt;
                latest_state.safety_state = safety_auditor.getState();
                latest_state.flags.distress_sos_active = safety_auditor.isDistressActive();
                latest_state.flags.panic_circling = safety_auditor.isPanicCirclingWarning();
            }
        }

        total_distance_m += stride_len_m;
        total_step_count++;

        trail_history.emplace_back(true_easting_m, true_northing_m);
        if (trail_history.size() > 500) {
            trail_history.erase(trail_history.begin());
        }

        series.maybeSample(latest_state.global_navigation_score,
                           latest_state.instantaneous_speed_m_s,
                           latest_state.vertical_speed_m_s,
                           latest_state.activity_confidence,
                           sim_altitude_m, current_time_ns);
    }

    void autoWalkTick(double wall_dt_s) {
        // Pacing đúng vật lý: bước mỗi step_interval_s theo wall-clock
        sim_wall_accum += wall_dt_s;
        while (sim_wall_accum >= step_interval_s) {
            sim_wall_accum -= step_interval_s;
            const double dx = true_easting_m - 1000.0;
            const double dy = true_northing_m - 1000.0;
            const double cur_heading = std::atan2(dx, dy);
            const double target_heading = cur_heading + 3.14159265358979323846 / 2.0;
            const double d_head = math::normalizeAngleRad(target_heading - true_heading_rad);
            simulateStep(0.75, std::clamp(d_head, -0.08, 0.08));
        }
    }

    void triggerFallEvent() {
        current_time_ns += 100000000ULL;
        safety_auditor.processMotionSample(Vector3f{0, 0, 9.81F}, Vector3f{4.5F, 1.2F, 3.2F}, current_time_ns);
        current_time_ns += 200000000ULL;
        safety_auditor.processMotionSample(Vector3f{0, 0, 48.0F}, Vector3f{0.5F, 0.2F, 0.1F}, current_time_ns);
        latest_state.safety_state = safety_auditor.getState();
    }

    void updateRescuePacket() {
        const auto& c0 = latest_state.clusters[0];
        const auto [lat, lon] = geo_origin.enToLatLon(c0.center_easting_m, c0.center_northing_m);
        const auto raw = Packet160Serializer::encode(
            static_cast<uint32_t>(current_time_ns / 1000000000ULL),
            lat, lon, sim_altitude_m,
            c0.ce95_semi_major_m * 0.5, c0.ce95_semi_minor_m * 0.5,
            c0.ellipse_heading_rad * 180.0 / 3.14159265358979323846,
            0x01, static_cast<uint8_t>(battery_soc), 1);
        OutboundRescuePacket160 out{};
        out.data = raw;
        const auto sms = SosEgressManager::buildCompactSmsPayload(out, latest_state, battery_soc);
        RescuePacketRecord rec{};
        rec.hex = toHexDump(raw.data(), raw.size());
        rec.sms.assign(sms.begin(), sms.end());
        rec.lat_deg = lat;
        rec.lon_deg = lon;
        rec.ce95_m = c0.ce95_semi_major_m;
        rec.battery = static_cast<int>(battery_soc);
        rescue_history.push_back(std::move(rec));
        while (rescue_history.size() > 10) rescue_history.pop_front();
    }
};

} // namespace desktop
