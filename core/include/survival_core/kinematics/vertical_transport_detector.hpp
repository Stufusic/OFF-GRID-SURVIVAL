#pragma once

#include "survival_core/types.hpp"
#include <cstdint>
#include <array>
#include <cmath>

namespace survival::kinematics {

struct VerticalTransportScores {
    float escalator_score{0.0F};
    float elevator_score{0.0F};
    float cable_score{0.0F};
    float cycling_score{0.0F};
    float vertical_velocity_m_s{0.0F};
};

class VerticalTransportDetector {
public:
    VerticalTransportDetector() noexcept;

    /**
     * @brief Xử lý mẫu và tính điểm các loại phương tiện đặc thù
     * @param baro_vertical_rate_m_s Tốc độ đổi độ cao từ áp kế (dz/dt)
     * @param vertical_accel_m_s2 Gia tốc thẳng đứng
     * @param human_cadence_hz Nhịp bước người
     * @param horizontal_speed_m_s Vận tốc ngang (từ GPS hoặc PDR)
     * @param hf_vibration_ratio Tỷ số rung động cao tần
     * @param dt_s Bước thời gian [s]
     */
    VerticalTransportScores processSample(float baro_vertical_rate_m_s, float vertical_accel_m_s2, float human_cadence_hz, float horizontal_speed_m_s, float hf_vibration_ratio, double dt_s) noexcept;

    void reset() noexcept;

private:
    float filtered_vz_{0.0F};
    float elevator_phase_time_s_{0.0F};
    uint8_t elevator_state_{0}; // 0: Idle, 1: Accelerating, 2: Constant Speed, 3: Decelerating
};

} // namespace survival::kinematics
