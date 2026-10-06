#pragma once

#include "survival_core/types.hpp"
#include <cmath>

namespace survival::kinematics {

struct VehicleKinematicState {
    double speed_m_s{0.0};
    double heading_rad{0.0};
    double yaw_rate_rad_s{0.0};
    double acceleration_m_s2{0.0};
    Vector3d velocity_enu_m_s{};
};

class VehicleMotionModel {
public:
    VehicleMotionModel() noexcept;

    /**
     * @brief Cập nhật trạng thái động học xe qua 1 bước thời gian (100 Hz)
     * @param heading_rad Góc hướng phương vị xe (từ ESKF)
     * @param yaw_rate_rad_s Tốc độ góc quay trục thẳng đứng
     * @param forward_accel_m_s2 Gia tốc dọc trục tiến của xe (đã bù trọng lực)
     * @param gnss_speed_m_s Vận tốc từ GNSS Doppler (< 0 nếu mất GPS)
     * @param is_stopped Cờ báo hiệu xe đang dừng đỗ
     * @param dt_s Bước thời gian [s]
     */
    VehicleKinematicState update(double heading_rad, double yaw_rate_rad_s, double forward_accel_m_s2, float gnss_speed_m_s, bool is_stopped, double dt_s) noexcept;

    void reset() noexcept;

    [[nodiscard]] double getCurrentSpeed() const noexcept {
        return current_speed_m_s_;
    }

private:
    double current_speed_m_s_{0.0};
    double current_heading_rad_{0.0};
};

} // namespace survival::kinematics
