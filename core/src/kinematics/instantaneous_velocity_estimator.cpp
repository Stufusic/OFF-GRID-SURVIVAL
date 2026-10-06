#include "survival_core/kinematics/instantaneous_velocity_estimator.hpp"
#include <algorithm>

namespace survival::kinematics {

namespace {

inline double computeAlpha(double cutoff_hz, double dt_s) noexcept {
    const double tau = 1.0 / (kTwoPi * cutoff_hz);
    return 1.0 / (1.0 + tau / dt_s);
}

} // namespace

InstantaneousVelocityEstimator::InstantaneousVelocityEstimator() noexcept {
    reset();
}

void InstantaneousVelocityEstimator::reset() noexcept {
    vehicle_model_.reset();
    filtered_speed_m_s_ = 0.0;
    filtered_vz_m_s_ = 0.0;
    prev_raw_speed_m_s_ = 0.0;
    one_euro_x_ = 0.0;
    one_euro_dx_ = 0.0;
}

double InstantaneousVelocityEstimator::filterOneEuro(double raw_val, double dt_s) noexcept {
    if (dt_s <= 0.0 || dt_s > 0.5) {
        return one_euro_x_;
    }

    constexpr double min_cutoff_hz = 0.8;
    constexpr double beta = 0.05;
    constexpr double d_cutoff_hz = 1.0;

    // 1. Tính toán đạo hàm vận tốc (dx)
    const double raw_dx = (raw_val - prev_raw_speed_m_s_) / dt_s;
    prev_raw_speed_m_s_ = raw_val;

    const double alpha_d = computeAlpha(d_cutoff_hz, dt_s);
    one_euro_dx_ = alpha_d * raw_dx + (1.0 - alpha_d) * one_euro_dx_;

    // 2. Tính tần số cắt động lực học
    const double cutoff_hz = min_cutoff_hz + beta * std::abs(one_euro_dx_);
    const double alpha = computeAlpha(cutoff_hz, dt_s);

    one_euro_x_ = alpha * raw_val + (1.0 - alpha) * one_euro_x_;
    return one_euro_x_;
}

VelocityEstimateOutput InstantaneousVelocityEstimator::update(
    MotionActivity activity,
    double heading_rad,
    double yaw_rate_rad_s,
    double fwd_accel_m_s2,
    double stride_length_m,
    double cadence_hz,
    float estimated_riser_m,
    float baro_vertical_rate_m_s,
    float gnss_speed_m_s,
    bool is_vehicle_stopped,
    double dt_s
) noexcept {
    VelocityEstimateOutput out{};

    if (dt_s <= 0.0 || dt_s > 0.5) {
        return out;
    }

    double raw_speed = 0.0;
    double raw_vz = 0.0;

    switch (activity) {
        case MotionActivity::kStationary:
            raw_speed = 0.0;
            raw_vz = 0.0;
            break;

        case MotionActivity::kWalkingFlat:
        case MotionActivity::kRunning: {
            const double L = (stride_length_m > 0.1) ? stride_length_m : 0.70;
            const double f = (cadence_hz > 0.1) ? cadence_hz : 0.0;
            raw_speed = L * f;
            raw_vz = static_cast<double>(baro_vertical_rate_m_s);
            break;
        }

        case MotionActivity::kStairsAscent: {
            constexpr double kTreadWidthM = kStairTreadM;  // [A1] thống nhất
            const double f = (cadence_hz > 0.1) ? cadence_hz : 1.5;
            raw_speed = kTreadWidthM * f;
            raw_vz = static_cast<double>(estimated_riser_m) * f;
            break;
        }

        case MotionActivity::kStairsDescent: {
            constexpr double kTreadWidthM = kStairTreadM;  // [A1] thống nhất
            const double f = (cadence_hz > 0.1) ? cadence_hz : 1.5;
            raw_speed = kTreadWidthM * f;
            raw_vz = -static_cast<double>(estimated_riser_m) * f;
            break;
        }

        case MotionActivity::kStairLanding: {
            const double L = (stride_length_m > 0.1) ? stride_length_m : 0.65;
            const double f = (cadence_hz > 0.1) ? cadence_hz : 1.5;
            raw_speed = L * f;
            raw_vz = 0.0;
            break;
        }

        case MotionActivity::kEscalator: {
            raw_speed = 0.50; // Vận tốc thang cuốn tiêu chuẩn
            raw_vz = static_cast<double>(baro_vertical_rate_m_s);
            break;
        }

        case MotionActivity::kElevator: {
            raw_speed = 0.0;
            raw_vz = static_cast<double>(baro_vertical_rate_m_s);
            break;
        }

        case MotionActivity::kVehicleWheeled: {
            const auto veh_state = vehicle_model_.update(heading_rad, yaw_rate_rad_s, fwd_accel_m_s2, gnss_speed_m_s, is_vehicle_stopped, dt_s);
            raw_speed = veh_state.speed_m_s;
            raw_vz = static_cast<double>(baro_vertical_rate_m_s);
            break;
        }

        case MotionActivity::kCycling: {
            const double f = (cadence_hz > 0.1) ? cadence_hz : 1.5;
            raw_speed = std::max(3.0, (gnss_speed_m_s > 0.0F) ? static_cast<double>(gnss_speed_m_s) : 3.5 * f);
            raw_vz = static_cast<double>(baro_vertical_rate_m_s);
            break;
        }

        default:
            raw_speed = 0.0;
            raw_vz = 0.0;
            break;
    }

    // 2. Làm mượt thích ứng qua bộ lọc One-Euro ở 100 Hz
    if (activity == MotionActivity::kStationary) {
        filtered_speed_m_s_ = 0.0;
        filtered_vz_m_s_ = 0.0;
        one_euro_x_ = 0.0;
        one_euro_dx_ = 0.0;
    } else {
        filtered_speed_m_s_ = filterOneEuro(raw_speed, dt_s);
        filtered_vz_m_s_ = 0.80 * filtered_vz_m_s_ + 0.20 * raw_vz;
    }

    out.horizontal_speed_m_s = filtered_speed_m_s_;
    out.vertical_speed_m_s = filtered_vz_m_s_;

    // Chiếu lên hệ Navigation ENU: East = v * sin(psi), North = v * cos(psi), Up = vz
    out.velocity_enu_m_s.x = filtered_speed_m_s_ * std::sin(heading_rad);
    out.velocity_enu_m_s.y = filtered_speed_m_s_ * std::cos(heading_rad);
    out.velocity_enu_m_s.z = filtered_vz_m_s_;
    out.speed_confidence = 1.0;

    return out;
}

} // namespace survival::kinematics
