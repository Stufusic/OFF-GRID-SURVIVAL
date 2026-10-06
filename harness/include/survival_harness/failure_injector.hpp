#pragma once

#include <cstdint>
#include <cmath>
#include <random>
#include <optional>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"

namespace survival::harness {

enum class ScenarioType : uint8_t {
    kM0_Nominal = 0,            // Dữ liệu chuẩn, nhiễu nền Gauss
    kM1_MagneticSpike = 1,      // Nhiễu xung từ trường +45 uT bên sườn
    kM2_BarometricDrift = 2,    // Trôi dạt áp suất khí quyển 2.0 hPa/giờ do bão
    kM3_CelestialOutage = 3,    // Mất quan sát Mặt Trời / Sương mù che khuất
    kM4_PdrNoiseInflation = 4,  // [M12] Nhiễu IMU: accel ±1.5 m/s² + gyro yaw ±0.15 rad/s (tiêm trong injectSensorFrame); injectStepEvent giữ cho test đơn vị
    kM_CombinedStress = 5       // Phối hợp đồng thời M1 + M2 + M3 + M4
};

struct FailureConfig {
    ScenarioType scenario{ScenarioType::kM0_Nominal};
    
    // M1: Magnetic Spike
    uint64_t mag_spike_start_ns{10ULL * 1000000000ULL}; // Bắt đầu ở giây thứ 10
    uint64_t mag_spike_end_ns{40ULL * 1000000000ULL};   // Kết thúc ở giây thứ 40
    double mag_spike_magnitude_uT{45.0};                // Cường độ +45 uT
    
    // M2: Barometric Drift
    uint64_t baro_drift_start_ns{0ULL};
    double baro_drift_hpa_per_hour{2.0};                // Giảm 2.0 hPa/giờ
    
    // M3: Celestial Outage
    uint64_t celestial_outage_start_ns{0ULL};
    uint64_t celestial_outage_end_ns{UINT64_MAX};       // Mất hoàn toàn
    
    // M4: PDR Noise Inflation
    double stride_corruption_std{0.25};                 // 25% biến thiên sải bước
    double heading_corruption_std_rad{0.20};            // ~11.5 độ biến thiên góc lái
    
    uint32_t environment_seed{1337};                    // Seed 1: Khóa nhiễu môi trường
};

class FailureInjector {
public:
    explicit FailureInjector(const FailureConfig& config = FailureConfig{}) noexcept;

    void setConfig(const FailureConfig& config) noexcept;
    void resetSeed(uint32_t seed) noexcept;

    /**
     * @brief Tiêm lỗi lên mẫu cảm biến SensorFrame theo đúng mô hình toán học vật lý
     */
    void injectSensorFrame(SensorFrame& frame) noexcept;

    /**
     * @brief Tiêm lỗi lên sự kiện bước chân StepEvent (kịch bản M4)
     */
    void injectStepEvent(StepEvent& step) noexcept;

    /**
     * @brief Kiểm tra xem tại thời điểm t có bị mất quan sát thiên văn không (kịch bản M3)
     */
    [[nodiscard]] bool isCelestialOutage(uint64_t timestamp_ns) const noexcept;

private:
    FailureConfig config_{};
    mutable std::mt19937 env_rng_;
};

} // namespace survival::harness
