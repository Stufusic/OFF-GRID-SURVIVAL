#include "survival_harness/synthetic_generator.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

namespace survival::harness {

SyntheticDataset SyntheticGenerator::generate(const SyntheticConfig& config) noexcept {
    SyntheticDataset dataset;

    std::mt19937_64 rng(config.random_seed);
    std::normal_distribution<double> dist_accel(0.0, config.accel_noise_std);
    std::normal_distribution<double> dist_gyro(0.0, config.gyro_noise_std);
    std::normal_distribution<double> dist_mag(0.0, config.mag_noise_std);
    std::normal_distribution<double> dist_baro(0.0, config.baro_noise_std);
    std::normal_distribution<double> dist_step(0.0, config.step_noise_std);

    const double dt = 1.0 / config.imu_rate_hz;
    const size_t total_imu_steps = static_cast<size_t>(std::ceil(config.duration_s * config.imu_rate_hz));
    const size_t mag_interval = static_cast<size_t>(std::max(1.0, std::round(config.imu_rate_hz / config.mag_rate_hz)));
    const size_t baro_interval = static_cast<size_t>(std::max(1.0, std::round(config.imu_rate_hz / config.baro_rate_hz)));
    const double step_period_s = 1.0 / config.step_cadence_hz;

    dataset.sensor_frames.reserve(total_imu_steps);
    dataset.ground_truth_positions.reserve(total_imu_steps);
    dataset.ground_truth_headings_rad.reserve(total_imu_steps);
    dataset.ground_truth_timestamps_ns.reserve(total_imu_steps);

    double current_e = 0.0;
    double current_n = 0.0;
    double current_alt = 1000.0;
    double current_heading = 0.0; // 0 = North
    double next_step_time_s = step_period_s;

    // Tốc độ đi bộ trung bình v = cadence * stride
    const double walking_speed = config.step_cadence_hz * config.nominal_stride_length_m;

    for (size_t step_idx = 0; step_idx < total_imu_steps; ++step_idx) {
        const double t = static_cast<double>(step_idx) * dt;
        const uint64_t timestamp_ns = static_cast<uint64_t>(t * 1e9);

        // 1. Cập nhật quỹ đạo Ground Truth tùy theo kịch bản
        double gyro_z_truth = 0.0;
        double speed = walking_speed;

        switch (config.trajectory_type) {
            case TrajectoryType::kStraightNorth:
                current_heading = 0.0;
                gyro_z_truth = 0.0;
                break;
            case TrajectoryType::kTurnRight90:
                if (t < 25.0) {
                    current_heading = 0.0;
                    gyro_z_truth = 0.0;
                } else if (t >= 25.0 && t < 30.0) {
                    // Rẽ phải 90 độ (North -> East) trong 5 giây: rate = (PI/2)/5 = 0.314 rad/s (quay Z âm)
                    const double turn_rate = kHalfPi / 5.0;
                    gyro_z_truth = -turn_rate;
                    current_heading += turn_rate * dt;
                } else {
                    current_heading = kHalfPi;
                    gyro_z_truth = 0.0;
                }
                break;
            case TrajectoryType::kCirclingLoop:
                // Đi vòng cung liên tục: bán kính R = 25m, chu kỳ T = 2*PI*R / v
                gyro_z_truth = -(walking_speed / 25.0);
                current_heading += (walking_speed / 25.0) * dt;
                break;
            case TrajectoryType::kMountainTrail:
                current_heading = 0.2 * std::sin(0.1 * t);
                gyro_z_truth = -0.02 * std::cos(0.1 * t);
                current_alt = 1000.0 + 1.5 * t; // Leo dốc 1.5 m/s
                break;
        }

        current_heading = math::normalizeHeadingRad(current_heading);

        // Dịch chuyển vị trí: East = speed * sin(heading), North = speed * cos(heading)
        current_e += speed * std::sin(current_heading) * dt;
        current_n += speed * std::cos(current_heading) * dt;

        dataset.ground_truth_positions.push_back({current_e, current_n, current_alt});
        dataset.ground_truth_headings_rad.push_back(current_heading);
        dataset.ground_truth_timestamps_ns.push_back(timestamp_ns);

        // 2. Tạo SensorFrame (Multi-rate)
        SensorFrame frame{};
        frame.timestamp_ns = timestamp_ns;
        frame.has_imu = true;

        // Gia tốc: Trọng trường hướng lên trong body frame + dao động nhịp bước + nhiễu
        const double vertical_walking_bounce = 2.5 * std::sin(kTwoPi * config.step_cadence_hz * t);
        frame.accel_m_s2[0] = static_cast<float>(dist_accel(rng));
        frame.accel_m_s2[1] = static_cast<float>(dist_accel(rng));
        frame.accel_m_s2[2] = static_cast<float>(kGravityStandard + vertical_walking_bounce + dist_accel(rng));

        // Gyro: Vận tốc góc xoay quanh trục Z
        frame.gyro_rad_s[0] = static_cast<float>(dist_gyro(rng));
        frame.gyro_rad_s[1] = static_cast<float>(dist_gyro(rng));
        frame.gyro_rad_s[2] = static_cast<float>(gyro_z_truth + dist_gyro(rng));

        // Từ kế (20 Hz): Vectơ từ trường Trái Đất chiếu theo heading (North ~ 30 uT, Up ~ -40 uT)
        // mx (Right) = -30*sin(psi), my (Forward) = 30*cos(psi), mz (Up) = -40
        if (step_idx % mag_interval == 0) {
            frame.has_mag = true;
            frame.mag_uT[0] = static_cast<float>(-30.0 * std::sin(current_heading) + dist_mag(rng));
            frame.mag_uT[1] = static_cast<float>(30.0 * std::cos(current_heading) + dist_mag(rng));
            frame.mag_uT[2] = static_cast<float>(-40.0 + dist_mag(rng));
        } else {
            frame.has_mag = false;
        }

        // Áp kế (10 Hz): Công thức khí áp vi sai P = P0 * (1 - alt / 44330)^5.255
        if (step_idx % baro_interval == 0) {
            frame.has_baro = true;
            const double expected_p = kStandardPressureHpa * std::pow(1.0 - current_alt / 44330.0, 5.255);
            frame.pressure_hpa = static_cast<float>(expected_p + dist_baro(rng));
        } else {
            frame.has_baro = false;
        }

        dataset.sensor_frames.push_back(frame);

        // 3. Tạo StepEvent khi đến chu kỳ bước chân
        if (t >= next_step_time_s) {
            StepEvent step{};
            step.timestamp_ns = timestamp_ns;
            step.stride_mean_m = config.nominal_stride_length_m + dist_step(rng);
            step.stride_variance_m2 = config.step_noise_std * config.step_noise_std;
            step.heading_rad = current_heading;
            step.heading_variance_rad2 = 0.01;
            step.vertical_displacement_m = (config.trajectory_type == TrajectoryType::kMountainTrail) ? 1.5 * step_period_s : 0.0;
            step.confidence = 1.0;

            dataset.step_events.push_back(step);
            next_step_time_s += step_period_s;
        }
    }

    return dataset;
}

} // namespace survival::harness
