#pragma once

#include "survival_core/types.hpp"
#include "survival_core/kinematics/vehicle_motion_model.hpp"
#include <cmath>

namespace survival::kinematics {

struct VelocityEstimateOutput {
    Vector3d velocity_enu_m_s{};
    double horizontal_speed_m_s{0.0};
    double vertical_speed_m_s{0.0};
    double speed_confidence{1.0};
};

class InstantaneousVelocityEstimator {
public:
    InstantaneousVelocityEstimator() noexcept;

    /**
     * @brief Cập nhật ước lượng vận tốc tức thời ở 100 Hz
     * @param activity Hoạt động đã được phân loại từ FSM
     * @param heading_rad Góc hướng phương vị (từ ESKF)
     * @param yaw_rate_rad_s Vận tốc góc quay trục Z
     * @param fwd_accel_m_s2 Gia tốc thân máy dọc trục tiến
     * @param stride_length_m Chiều dài sải bước ước lượng
     * @param cadence_hz Nhịp bước chân người
     * @param estimated_riser_m Chiều cao bậc thang ước lượng
     * @param baro_vertical_rate_m_s Tốc độ đổi độ cao khí áp
     * @param gnss_speed_m_s Vận tốc từ GNSS Doppler (< 0 nếu không có GPS)
     * @param is_vehicle_stopped Cờ xe dừng đỗ
     * @param dt_s Bước thời gian [s]
     */
    [[nodiscard]] VelocityEstimateOutput update(
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
    ) noexcept;

    void reset() noexcept;

    [[nodiscard]] double getFilteredSpeed() const noexcept {
        return filtered_speed_m_s_;
    }

private:
    VehicleMotionModel vehicle_model_{};
    double filtered_speed_m_s_{0.0};
    double filtered_vz_m_s_{0.0};
    double prev_raw_speed_m_s_{0.0};

    // Tham số bộ lọc thích ứng One-Euro
    double one_euro_x_{0.0};
    double one_euro_dx_{0.0};

    [[nodiscard]] double filterOneEuro(double raw_val, double dt_s) noexcept;
};

} // namespace survival::kinematics
