#include "survival_core/kinematics/stair_geometry_estimator.hpp"

namespace survival::kinematics {

StairGeometryEstimator::StairGeometryEstimator(const StairGeometryConfig& config) noexcept
    : config_(config)
{
    reset();
}

void StairGeometryEstimator::reset() noexcept {
    state_ = StairState{};
    state_.estimated_riser_m = config_.default_riser_m;
    state_.confidence = 0.0F;
    state_.is_on_landing = false;
    consecutive_flight_steps_ = 0;
    valid_baro_delta_count_ = 0;
    flight_baro_accum_m_ = 0.0;
}

void StairGeometryEstimator::recordStairStep(bool is_ascent, double delta_baro_alt_m, bool has_baro, double step_interval_s) noexcept {
    state_.step_count++;
    consecutive_flight_steps_++;
    state_.is_on_landing = false;

    if (step_interval_s > 0.1 && step_interval_s < 2.0) {
        state_.cadence_hz = static_cast<float>(1.0 / step_interval_s);
    }

    if (has_baro && std::abs(delta_baro_alt_m) > 0.02 && std::abs(delta_baro_alt_m) < 0.50) {
        flight_baro_accum_m_ += std::abs(delta_baro_alt_m);
        valid_baro_delta_count_++;

        if (valid_baro_delta_count_ >= config_.min_steps_for_lock) {
            // [M7] Chia theo số delta HỢP LỆ (trước: chia theo tổng bước trong chặng
            // khiến avg bị kéo lệch thấp khi baro rơi mẫu giữa chặng)
            const float avg_riser = static_cast<float>(flight_baro_accum_m_ / static_cast<double>(valid_baro_delta_count_));
            const float clamped_riser = std::clamp(avg_riser, config_.min_valid_riser_m, config_.max_valid_riser_m);
            // Lọc mượt thích ứng EMA
            state_.estimated_riser_m = 0.7F * state_.estimated_riser_m + 0.3F * clamped_riser;
            state_.confidence = std::min(1.0F, state_.confidence + 0.2F);
        }
    } else {
        // Không có áp kế -> tăng confidence theo nhịp bước ổn định
        state_.confidence = std::min(0.85F, state_.confidence + 0.1F);
    }

    const float delta_z = is_ascent ? state_.estimated_riser_m : -state_.estimated_riser_m;
    state_.cumulative_vertical_m += delta_z;
}

void StairGeometryEstimator::recordLandingStep() noexcept {
    state_.landing_count++;
    state_.is_on_landing = true;
    consecutive_flight_steps_ = 0;
    valid_baro_delta_count_ = 0;
    flight_baro_accum_m_ = 0.0;
}

} // namespace survival::kinematics
