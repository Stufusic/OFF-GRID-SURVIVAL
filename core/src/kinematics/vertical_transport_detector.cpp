#include "survival_core/kinematics/vertical_transport_detector.hpp"
#include <algorithm>

namespace survival::kinematics {

VerticalTransportDetector::VerticalTransportDetector() noexcept {
    reset();
}

void VerticalTransportDetector::reset() noexcept {
    filtered_vz_ = 0.0F;
    elevator_phase_time_s_ = 0.0F;
    elevator_state_ = 0;
}

VerticalTransportScores VerticalTransportDetector::processSample(float baro_vertical_rate_m_s, float vertical_accel_m_s2, float human_cadence_hz, float horizontal_speed_m_s, float hf_vibration_ratio, double dt_s) noexcept {
    VerticalTransportScores scores{};

    if (dt_s <= 0.0 || dt_s > 0.5) {
        return scores;
    }

    filtered_vz_ = 0.90F * filtered_vz_ + 0.10F * baro_vertical_rate_m_s;
    const float abs_vz = std::abs(filtered_vz_);
    scores.vertical_velocity_m_s = filtered_vz_;

    const bool no_human_steps = (human_cadence_hz < 0.3F);

    // 1. Thang cuốn (Escalator):
    // Cadence ~ 0, tốc độ lên/xuống đều đặn 0.4 - 0.9 m/s, rung động cơ thấp
    if (no_human_steps && abs_vz >= 0.35F && abs_vz <= 1.0F) {
        if (horizontal_speed_m_s >= 0.2F && horizontal_speed_m_s <= 1.5F) {
            scores.escalator_score = 0.85F;
        } else {
            scores.escalator_score = 0.60F;
        }
    }

    // 2. Thang máy (Elevator):
    // Cadence ~ 0, tốc độ lên/xuống nhanh > 1.0 m/s đến 6.0 m/s
    if (no_human_steps && abs_vz > 1.0F) {
        const float vz_factor = std::clamp((abs_vz - 1.0F) / 3.0F, 0.0F, 1.0F);
        scores.elevator_score = 0.70F + 0.25F * vz_factor;
    }

    // 3. Cáp treo (Cable Transport):
    // Tốc độ ngang > 2.0 m/s kèm theo tốc độ đứng vz > 0.5 m/s
    if (no_human_steps && horizontal_speed_m_s > 2.0F && abs_vz > 0.5F) {
        scores.cable_score = 0.80F;
    }

    // 4. Xe đạp (Cycling):
    // Nhịp guồng chân đều 1.0 - 2.0 Hz, gia tốc giật nảy thấp (không có va đập gót chân P2P lớn), tốc độ ngang 3.0 - 12.0 m/s
    if (human_cadence_hz >= 0.8F && human_cadence_hz <= 2.2F) {
        if (horizontal_speed_m_s >= 3.0F && horizontal_speed_m_s <= 15.0F && hf_vibration_ratio < 1.0F) {
            scores.cycling_score = 0.80F;
        }
    }

    return scores;
}

} // namespace survival::kinematics
