#include "survival_core/kinematics/motion_transition_fsm.hpp"
#include <cmath>

namespace survival::kinematics {

MotionTransitionFsm::MotionTransitionFsm(const FsmConfig& config) noexcept
    : config_(config)
{
    reset();
}

void MotionTransitionFsm::reset() noexcept {
    current_activity_ = MotionActivity::kStationary;
    candidate_activity_ = MotionActivity::kStationary;
    temporal_scores_.fill(0.0F);
    temporal_scores_[static_cast<size_t>(MotionActivity::kStationary)] = 1.0F;
    time_in_current_state_s_ = 0.0;
    current_fused_confidence_ = 1.0F;
}

bool MotionTransitionFsm::isValidTransition(MotionActivity from, MotionActivity to) noexcept {
    if (from == to) return true;

    switch (from) {
        case MotionActivity::kStationary:
            return (to == MotionActivity::kWalkingFlat ||
                    to == MotionActivity::kRunning ||
                    to == MotionActivity::kStairsAscent ||
                    to == MotionActivity::kStairsDescent ||
                    to == MotionActivity::kVehicleWheeled ||
                    to == MotionActivity::kCycling ||
                    to == MotionActivity::kElevator ||
                    to == MotionActivity::kEscalator);

        case MotionActivity::kWalkingFlat:
            return (to == MotionActivity::kStationary ||
                    to == MotionActivity::kRunning ||
                    to == MotionActivity::kStairsAscent ||
                    to == MotionActivity::kStairsDescent ||
                    to == MotionActivity::kEscalator ||
                    to == MotionActivity::kCycling ||
                    to == MotionActivity::kVehicleWheeled);

        case MotionActivity::kRunning:
            return (to == MotionActivity::kWalkingFlat ||
                    to == MotionActivity::kStationary ||
                    to == MotionActivity::kStairsAscent ||
                    to == MotionActivity::kStairsDescent);

        case MotionActivity::kStairsAscent:
            return (to == MotionActivity::kStairLanding ||
                    to == MotionActivity::kWalkingFlat ||
                    to == MotionActivity::kStationary);

        case MotionActivity::kStairsDescent:
            return (to == MotionActivity::kStairLanding ||
                    to == MotionActivity::kWalkingFlat ||
                    to == MotionActivity::kStationary);

        case MotionActivity::kStairLanding:
            return (to == MotionActivity::kStairsAscent ||
                    to == MotionActivity::kStairsDescent ||
                    to == MotionActivity::kWalkingFlat ||
                    to == MotionActivity::kStationary);

        case MotionActivity::kVehicleWheeled:
            return (to == MotionActivity::kStationary ||
                    to == MotionActivity::kWalkingFlat);

        case MotionActivity::kEscalator:
            return (to == MotionActivity::kWalkingFlat ||
                    to == MotionActivity::kStationary);

        case MotionActivity::kElevator:
            return (to == MotionActivity::kStationary ||
                    to == MotionActivity::kWalkingFlat);

        case MotionActivity::kCycling:
            return (to == MotionActivity::kStationary ||
                    to == MotionActivity::kWalkingFlat);

        default:
            return true;
    }
}

MotionActivityEstimate MotionTransitionFsm::update(const std::array<float, 12>& candidate_scores, double dt_s) noexcept {
    MotionActivityEstimate est{};

    if (dt_s <= 0.0 || dt_s > 0.5) {
        est.current_activity = current_activity_;
        est.fused_confidence = current_fused_confidence_;
        return est;
    }

    time_in_current_state_s_ += dt_s;

    // 1. Cập nhật điểm thời gian tích lũy (Temporal Exponential Smoothing)
    size_t best_candidate_idx = 0;
    float max_raw_score = 0.0F;

    for (size_t i = 0; i < 12; ++i) {
        temporal_scores_[i] = config_.temporal_decay_rate * temporal_scores_[i] + (1.0F - config_.temporal_decay_rate) * candidate_scores[i];
        if (candidate_scores[i] > max_raw_score) {
            max_raw_score = candidate_scores[i];
            best_candidate_idx = i;
        }
    }

    candidate_activity_ = static_cast<MotionActivity>(best_candidate_idx);
    const float candidate_temporal_score = temporal_scores_[best_candidate_idx];
    const float fused_candidate_score = 0.40F * max_raw_score + 0.60F * candidate_temporal_score;

    const size_t current_idx = static_cast<size_t>(current_activity_);
    const float current_state_temporal = temporal_scores_[current_idx];

    // 2. Kiểm tra điều kiện chuyển đổi có trễ (Hysteresis Transition Rule)
    if (candidate_activity_ != current_activity_) {
        const bool valid_graph_transition = isValidTransition(current_activity_, candidate_activity_);
        const bool dwell_satisfied = (time_in_current_state_s_ >= config_.min_dwell_time_s);
        const bool is_stair_target = (candidate_activity_ == MotionActivity::kStairsAscent ||
                                      candidate_activity_ == MotionActivity::kStairsDescent);
        const bool high_confidence_override = (fused_candidate_score >= 0.90F && !is_stair_target);

        if (valid_graph_transition && (dwell_satisfied || high_confidence_override)) {
            // [A6] ĐÃ THỬ nới trần temporal cho mục tiêu stair (0.90): nhãn stair
            // tăng 39s -> 67s trên D1 nhưng vertical DỮ hơn (Z_end +1.98 -> −4.79)
            // do trung censor walking-as-stairs — đo a6d. Giữ gate nghiêm: nhãn
            // stair hẹp nhưng ĐÚNG hướng (D1 Z_end +1.98, loop 2.31m).
            if (fused_candidate_score >= config_.threshold_enter && current_state_temporal < config_.threshold_exit + 0.20F) {
                // CHUYỂN TRẠNG THÁI HỢP LỆ
                current_activity_ = candidate_activity_;
                time_in_current_state_s_ = 0.0;
            }
        }
    }

    current_fused_confidence_ = std::clamp(temporal_scores_[static_cast<size_t>(current_activity_)], 0.0F, 1.0F);

    est.current_activity = current_activity_;
    est.candidate_activity = candidate_activity_;
    est.raw_confidence = max_raw_score;
    est.temporal_confidence = candidate_temporal_score;
    est.fused_confidence = current_fused_confidence_;

    return est;
}

} // namespace survival::kinematics
