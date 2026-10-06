#pragma once

#include "survival_core/types.hpp"
#include <cstdint>
#include <array>
#include <algorithm>

namespace survival::kinematics {

struct FsmConfig {
    float threshold_enter{0.75F};       // Ngưỡng tin cậy để chuyển sang trạng thái mới
    float threshold_exit{0.35F};        // Ngưỡng tin cậy tối thiểu để giữ trạng thái hiện tại
    float temporal_decay_rate{0.92F};   // Tốc độ suy giảm lịch sử thời gian
    double min_dwell_time_s{0.80};      // Thời gian duy trì tối thiểu [s] chống đổi trạng thái liên tục
};

class MotionTransitionFsm {
public:
    explicit MotionTransitionFsm(const FsmConfig& config = FsmConfig{}) noexcept;

    /**
     * @brief Cập nhật máy trạng thái với điểm số của các ứng viên hoạt động
     * @param candidate_scores Mảng điểm tin cậy của 11 trạng thái hoạt động
     * @param dt_s Bước thời gian [s]
     * @return Hoạt động đã được chốt và độ tin cậy kết hợp
     */
    [[nodiscard]] MotionActivityEstimate update(const std::array<float, 12>& candidate_scores, double dt_s) noexcept;

    void reset() noexcept;

    [[nodiscard]] MotionActivity getCurrentActivity() const noexcept {
        return current_activity_;
    }

    /// [A6] Đồng bộ buộc trạng thái (bypass graph/hysteresis): chỉ dùng cho các
    /// sự kiện bằng chứng mạnh từ detector — đảo hướng trong phiên thang
    /// (3 bước liên tiếp lệch cụm skewness). Reset temporal để nhãn mới không
    /// bị nhớ cũ kéo ngược.
    void forceActivity(MotionActivity activity) noexcept {
        current_activity_ = activity;
        candidate_activity_ = activity;
        temporal_scores_.fill(0.0F);
        time_in_current_state_s_ = 0.0;
    }

    [[nodiscard]] static bool isValidTransition(MotionActivity from, MotionActivity to) noexcept;

private:
    FsmConfig config_{};
    MotionActivity current_activity_{MotionActivity::kStationary};
    MotionActivity candidate_activity_{MotionActivity::kStationary};
    
    std::array<float, 12> temporal_scores_{};
    double time_in_current_state_s_{0.0};
    float current_fused_confidence_{1.0F};
};

} // namespace survival::kinematics
