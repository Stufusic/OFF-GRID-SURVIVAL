#include "survival_core/kinematics/vehicle_motion_model.hpp"
#include <algorithm>

namespace survival::kinematics {

VehicleMotionModel::VehicleMotionModel() noexcept {
    reset();
}

void VehicleMotionModel::reset() noexcept {
    current_speed_m_s_ = 0.0;
    current_heading_rad_ = 0.0;
}

VehicleKinematicState VehicleMotionModel::update(double heading_rad, double yaw_rate_rad_s, double forward_accel_m_s2, float gnss_speed_m_s, bool is_stopped, double dt_s) noexcept {
    VehicleKinematicState state{};

    if (dt_s <= 0.0 || dt_s > 0.5) {
        return state;
    }

    current_heading_rad_ = heading_rad;

    if (is_stopped) {
        current_speed_m_s_ = 0.0;
    } else if (gnss_speed_m_s >= 0.0F) {
        // Có GPS: Đồng bộ hóa chặt chẽ theo GPS Doppler
        current_speed_m_s_ = 0.85 * static_cast<double>(gnss_speed_m_s) + 0.15 * (current_speed_m_s_ + forward_accel_m_s2 * dt_s);
    } else {
        // Mất GPS: Tích phân động học có lực cản suy giảm tự nhiên (Unicycle Model with Drag)
        // a_eff = a_fwd - k_drag * v
        constexpr double kDragCoeff = 0.05; // Hệ số cản hãm trôi phân kỳ
        const double a_eff = std::clamp(forward_accel_m_s2, -8.0, 5.0) - kDragCoeff * current_speed_m_s_;
        current_speed_m_s_ = std::max(0.0, current_speed_m_s_ + a_eff * dt_s);
    }

    // Giới hạn vận tốc xe hợp lý (0 -> 60 m/s ~ 216 km/h)
    current_speed_m_s_ = std::clamp(current_speed_m_s_, 0.0, 60.0);

    state.speed_m_s = current_speed_m_s_;
    state.heading_rad = current_heading_rad_;
    state.yaw_rate_rad_s = yaw_rate_rad_s;
    state.acceleration_m_s2 = forward_accel_m_s2;

    // Chiếu vận tốc lên hệ Navigation ENU: East = v * sin(psi), North = v * cos(psi)
    state.velocity_enu_m_s.x = current_speed_m_s_ * std::sin(current_heading_rad_);
    state.velocity_enu_m_s.y = current_speed_m_s_ * std::cos(current_heading_rad_);
    state.velocity_enu_m_s.z = 0.0;

    return state;
}

} // namespace survival::kinematics
