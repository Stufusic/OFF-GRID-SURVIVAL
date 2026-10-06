#include "survival_core/kinematics/baseline_stride_models.hpp"
#include <cmath>
#include <algorithm>

namespace survival::kinematics {

StrideEstimate WeinbergModel::estimateStride(const StepInputFeatures& features) const noexcept {
    const double delta_a = std::clamp(features.vertical_accel_p2p, 0.1, 40.0);
    // L = K * (Delta a)^(1/4)
    const double length = k_ * std::sqrt(std::sqrt(delta_a));
    const double clamped_len = std::clamp(length, 0.2, 1.8);
    return StrideEstimate{.mean_length_m = clamped_len, .variance_m2 = 0.0064}; // sigma = 0.08m
}

StrideEstimate KimModel::estimateStride(const StepInputFeatures& features) const noexcept {
    const double a_rms = std::clamp(features.vertical_accel_rms, 0.1, 20.0);
    // L = K * (RMS)^(1/3)
    const double length = k_ * std::cbrt(a_rms);
    const double clamped_len = std::clamp(length, 0.2, 1.8);
    return StrideEstimate{.mean_length_m = clamped_len, .variance_m2 = 0.0064};
}

StrideEstimate ScarlettModel::estimateStride(const StepInputFeatures& features) const noexcept {
    const double a_rms = std::clamp(features.vertical_accel_rms, 0.1, 20.0);
    const double delta_a = std::clamp(features.vertical_accel_p2p, 0.2, 40.0);
    // L = K * (a_rms / delta_a + 0.5)
    const double length = k_ * (a_rms / delta_a + 0.5);
    const double clamped_len = std::clamp(length, 0.2, 1.8);
    return StrideEstimate{.mean_length_m = clamped_len, .variance_m2 = 0.0064};
}

} // namespace survival::kinematics
