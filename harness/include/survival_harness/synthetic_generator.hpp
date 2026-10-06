#pragma once

#include <cstdint>
#include <vector>
#include <random>
#include "survival_core/data_contracts.hpp"
#include "survival_core/types.hpp"

namespace survival::harness {

enum class TrajectoryType : uint8_t {
    kStraightNorth = 0,    // Đi thẳng về hướng Bắc
    kTurnRight90 = 1,      // Đi Bắc 30s rồi rẽ Đông 30s
    kCirclingLoop = 2,     // Đi theo vòng tròn khép kín (Panic Circling)
    kMountainTrail = 3     // Đi đường đèo núi thay đổi cao độ dốc
};

struct SyntheticConfig {
    TrajectoryType trajectory_type{TrajectoryType::kStraightNorth};
    double duration_s{60.0};              // [s] Tổng thời gian mô phỏng
    double imu_rate_hz{100.0};            // [Hz] Tần số lấy mẫu IMU
    double mag_rate_hz{20.0};             // [Hz] Tần số lấy mẫu Từ kế
    double baro_rate_hz{10.0};            // [Hz] Tần số lấy mẫu Áp kế
    double step_cadence_hz{1.8};          // [Hz] Nhịp bước chân (bước/giây)
    double nominal_stride_length_m{0.75}; // [m] Chiều dài bước chuẩn
    
    // Nhiễu cảm biến Gauss
    double accel_noise_std{0.05};         // [m/s^2]
    double gyro_noise_std{0.005};         // [rad/s]
    double mag_noise_std{0.5};            // [uT]
    double baro_noise_std{0.2};           // [hPa]
    double step_noise_std{0.03};          // [m]
    
    uint64_t random_seed{42};             // Cố định seed cho Deterministic Replay
};

struct SyntheticDataset {
    std::vector<SensorFrame> sensor_frames;
    std::vector<StepEvent> step_events;
    std::vector<EastingNorthingAlt> ground_truth_positions;
    std::vector<double> ground_truth_headings_rad;
    std::vector<uint64_t> ground_truth_timestamps_ns;
};

class SyntheticGenerator {
public:
    [[nodiscard]] static SyntheticDataset generate(const SyntheticConfig& config) noexcept;
};

} // namespace survival::harness
