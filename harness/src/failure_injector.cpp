#include "survival_harness/failure_injector.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <algorithm>

namespace survival::harness {

FailureInjector::FailureInjector(const FailureConfig& config) noexcept
    : config_(config)
    , env_rng_(config.environment_seed)
{
}

void FailureInjector::setConfig(const FailureConfig& config) noexcept {
    config_ = config;
    env_rng_.seed(config_.environment_seed);
}

void FailureInjector::resetSeed(uint32_t seed) noexcept {
    config_.environment_seed = seed;
    env_rng_.seed(seed);
}

void FailureInjector::injectSensorFrame(SensorFrame& frame) noexcept {
    const uint64_t t_ns = frame.timestamp_ns;

    // 1. Kịch bản M1: Magnetic Spike (+45 uT)
    if (config_.scenario == ScenarioType::kM1_MagneticSpike || config_.scenario == ScenarioType::kM_CombinedStress) {
        if (t_ns >= config_.mag_spike_start_ns && t_ns <= config_.mag_spike_end_ns) {
            // Nhiễu từ trường cục bộ vuông góc với hướng di chuyển (trục Y body)
            frame.mag_uT[1] += static_cast<float>(config_.mag_spike_magnitude_uT);
        }
    }

    // 2. Kịch bản M2: Barometric Weather Drift (-2.0 hPa/giờ)
    if (config_.scenario == ScenarioType::kM2_BarometricDrift || config_.scenario == ScenarioType::kM_CombinedStress) {
        if (t_ns >= config_.baro_drift_start_ns) {
            const double delta_t_hours = static_cast<double>(t_ns - config_.baro_drift_start_ns) / (3600.0 * 1e9);
            const double delta_p = config_.baro_drift_hpa_per_hour * delta_t_hours;
            frame.pressure_hpa -= static_cast<float>(delta_p);
        }
    }

    // 3. Kịch bản M3: Celestial Outage
    if (config_.scenario == ScenarioType::kM3_CelestialOutage || config_.scenario == ScenarioType::kM_CombinedStress) {
        if (t_ns >= config_.celestial_outage_start_ns && t_ns <= config_.celestial_outage_end_ns) {
            frame.has_optical_sun = false;
            frame.optical_sun_azimuth_rad = -1.0F;
            frame.optical_confidence = 0.0F;
        }
    }

    // 4. Kịch bản M4: PDR Noise Inflation — [M12] tiêm trực tiếp lên IMU. Trước đây
    // M4 chỉ có injectStepEvent (không ai gọi trong benchmark) nên kịch bản M4 cho
    // kết quả giống hệt M0. Pipeline tự phát hiện bước chân nội bộ từ IMU nên nhiễu
    // gia tốc + gyro tại đây mới thực sự làm suy giảm PDR.
    if (config_.scenario == ScenarioType::kM4_PdrNoiseInflation || config_.scenario == ScenarioType::kM_CombinedStress) {
        if (frame.has_imu) {
            std::normal_distribution<double> dist_accel(0.0, 1.5);   // m/s^2
            std::normal_distribution<double> dist_gyro(0.0, 0.15);   // rad/s (~8.6 deg/s)
            frame.accel_m_s2[0] += static_cast<float>(dist_accel(env_rng_));
            frame.accel_m_s2[2] += static_cast<float>(dist_accel(env_rng_));
            frame.gyro_rad_s[2] += static_cast<float>(dist_gyro(env_rng_));
        }
    }
}

void FailureInjector::injectStepEvent(StepEvent& step) noexcept {
    // 4. Kịch bản M4: PDR Noise Inflation
    if (config_.scenario == ScenarioType::kM4_PdrNoiseInflation || config_.scenario == ScenarioType::kM_CombinedStress) {
        std::normal_distribution<double> dist_stride(0.0, config_.stride_corruption_std);
        std::normal_distribution<double> dist_head(0.0, config_.heading_corruption_std_rad);

        step.stride_mean_m = std::max(0.10, step.stride_mean_m * (1.0 + dist_stride(env_rng_)));
        step.heading_rad = math::normalizeHeadingRad(step.heading_rad + dist_head(env_rng_));
        step.stride_variance_m2 *= 4.0; // Phóng to phương sai để phản ánh bất định
        step.heading_variance_rad2 *= 4.0;
    }
}

bool FailureInjector::isCelestialOutage(uint64_t timestamp_ns) const noexcept {
    if (config_.scenario == ScenarioType::kM3_CelestialOutage || config_.scenario == ScenarioType::kM_CombinedStress) {
        return (timestamp_ns >= config_.celestial_outage_start_ns && timestamp_ns <= config_.celestial_outage_end_ns);
    }
    return false;
}

} // namespace survival::harness
