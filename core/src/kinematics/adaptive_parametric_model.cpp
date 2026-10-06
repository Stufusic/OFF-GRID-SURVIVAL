#include "survival_core/kinematics/adaptive_parametric_model.hpp"
#include <cmath>
#include <algorithm>

namespace survival::kinematics {

AdaptiveParametricModel::AdaptiveParametricModel(const AdaptiveModelParams& params) noexcept
    : params_(params)
{
}

double AdaptiveParametricModel::computeBiomechanicSlopeFactor(double slope_rad) noexcept {
    const double tan_slope = std::tan(slope_rad);
    double phi = 1.0;

    if (slope_rad >= 0.0) {
        // Khi leo dốc (slope >= 0): Suy giảm mạnh theo exp(-1.8 * tan(theta))
        phi = std::exp(-1.8 * tan_slope);
    } else {
        // Khi xuống dốc (slope < 0): Suy giảm theo exp(-0.8 * |tan(theta) + 0.15|)
        // Đi dốc thoai thoải xuống nhẹ (~8.5 độ -> tan=-0.15) là góc sải chân tối ưu phi ≈ 1.0
        phi = std::exp(-0.8 * std::abs(tan_slope + 0.15));
    }

    return std::clamp(phi, 0.10, 1.20);
}

StrideEstimate AdaptiveParametricModel::estimateStride(const StepInputFeatures& features) const noexcept {
    const double leg_length = (features.user_leg_length_m > 0.3) ? features.user_leg_length_m : 0.85;
    const double cadence = std::clamp(features.cadence_hz, 0.5, 3.5);
    const double delta_av = std::clamp(features.vertical_accel_p2p, 0.1, 30.0);

    // 1. Tính toán sải bước danh định theo nhịp và gia tốc đỉnh-đáy
    const double f_norm = cadence / params_.f0_hz;
    const double a_norm = std::cbrt(delta_av / params_.a0_m_s2);
    const double phi_slope = computeBiomechanicSlopeFactor(features.terrain_slope_rad);

    double stride_mean = leg_length * (params_.alpha * f_norm + params_.beta * a_norm) * phi_slope;
    stride_mean = std::clamp(stride_mean, 0.15, 2.0);

    // 2. Tính toán phương sai sải bước sigma_L^2
    const double tan_slope_abs = std::abs(std::tan(features.terrain_slope_rad));
    const double var_t_norm = std::max(0.0, features.step_period_var) / (params_.tau0_s * params_.tau0_s);

    const double sigma = params_.sigma0_m * (1.0 + params_.gamma1 * tan_slope_abs + params_.gamma2 * var_t_norm);
    const double clamped_sigma = std::clamp(sigma, 0.01, 0.50);

    return StrideEstimate{
        .mean_length_m = stride_mean,
        .variance_m2 = clamped_sigma * clamped_sigma
    };
}

} // namespace survival::kinematics
