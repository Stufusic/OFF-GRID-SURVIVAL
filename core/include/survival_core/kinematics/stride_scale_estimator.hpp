// ============================================================================
// [A2] StrideScaleEstimator — hiệu chuẩn hệ số sải bước K kiểu Weiberg
// ============================================================================
// Innovation = quãng đường GNSS giữa 2 anchor / quãng đường PDR cùng khoảng.
// EMA tiến về ratio với gain giảm dần theo |ratio-1| (tỉ lệ lệch càng lớn,
// càng nghi ngờ outlier nên cập nhật càng thận trọng). Clamp [min, max].
#pragma once

#include <algorithm>

namespace survival::kinematics {

class StrideScaleEstimator {
public:
    struct Config {
        double min_scale{0.70};
        double max_scale{1.40};
        double ema_gain{0.15};       // mỗi anchor hợp lệ
        double ratio_lo{0.50};       // ngoài dải này = outlier, bỏ lượt
        double ratio_hi{2.00};
    };

    StrideScaleEstimator() noexcept = default;
    explicit StrideScaleEstimator(const Config& config) noexcept
        : config_(config) {}

    /// Cập nhật với 1 innovation hợp lệ; trả về scale mới.
    /// EMA NHÂN tính đúng: innovation so với scale hiện hành (ratio/scale),
    /// hội tụ nhân về ratio thật mà không overshoot.
    double update(double ratio) noexcept {
        if (ratio < config_.ratio_lo || ratio > config_.ratio_hi) {
            return scale_;  // outlier — bỏ lượt
        }
        const double effective = ratio / std::max(1e-6, scale_);
        const double dev = std::abs(effective - 1.0);
        const double gain = config_.ema_gain * std::clamp(1.0 - (dev - 0.2) / 0.8, 0.2, 1.0);
        scale_ = std::clamp(scale_ * (1.0 + gain * (effective - 1.0)),
                            config_.min_scale, config_.max_scale);
        updates_++;
        return scale_;
    }

    [[nodiscard]] double scale() const noexcept { return scale_; }
    [[nodiscard]] uint32_t updateCount() const noexcept { return updates_; }
    void reset() noexcept { scale_ = 1.0; updates_ = 0; }

private:
    Config config_{};
    double scale_{1.0};
    uint32_t updates_{0};
};

} // namespace survival::kinematics
