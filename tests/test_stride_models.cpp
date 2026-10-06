#include <gtest/gtest.h>
#include "survival_core/kinematics/adaptive_parametric_model.hpp"
#include "survival_core/kinematics/baseline_stride_models.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

using namespace survival;
using namespace survival::kinematics;
using namespace survival::math;

TEST(StrideModelsTest, AdaptiveParametricFlatGround) {
    AdaptiveParametricModel model;
    StepInputFeatures features{
        .cadence_hz = 1.8,
        .vertical_accel_p2p = 3.0,
        .vertical_accel_rms = 1.0,
        .terrain_slope_rad = 0.0,
        .step_period_var = 0.005,
        .user_leg_length_m = 0.85
    };

    const auto est = model.estimateStride(features);
    // Đi bằng phẳng nhịp chuẩn: Sải bước kỳ vọng khoảng 0.70m - 0.75m
    EXPECT_GT(est.mean_length_m, 0.65);
    EXPECT_LT(est.mean_length_m, 0.80);
    // [C3c] sigma0 hiệu chỉnh 0.04 -> 0.12 m theo NEES MC (357 -> ~200, xem báo cáo)
    EXPECT_NEAR(est.variance_m2, 0.0144, 0.003); // sigma ~ 0.12m
}

TEST(StrideModelsTest, BiomechanicSlopeFactorAsymmetry) {
    // 1. Leo dốc 15 độ (0.2618 rad) -> Sải bước giảm mạnh khoảng 35% - 40% (phi ~ 0.62)
    const double slope_up_15 = degToRad(15.0);
    const double phi_up = AdaptiveParametricModel::computeBiomechanicSlopeFactor(slope_up_15);
    EXPECT_NEAR(phi_up, 0.617, 0.03); // exp(-1.8 * tan(15 deg)) ≈ 0.617
    EXPECT_LT(phi_up, 0.70);

    // 2. Xuống dốc nhẹ 8.5 độ (-0.148 rad, tan ≈ -0.15) -> Sải bước tối ưu phi ≈ 1.0
    const double slope_down_opt = degToRad(-8.5);
    const double phi_down_opt = AdaptiveParametricModel::computeBiomechanicSlopeFactor(slope_down_opt);
    EXPECT_NEAR(phi_down_opt, 1.0, 0.02);

    // 3. Xuống dốc gắt 30 độ (-0.523 rad, tan ≈ -0.577) -> Sải bước bị hãm lại
    const double slope_down_steep = degToRad(-30.0);
    const double phi_down_steep = AdaptiveParametricModel::computeBiomechanicSlopeFactor(slope_down_steep);
    EXPECT_LT(phi_down_steep, 0.75);
}

TEST(StrideModelsTest, BaselineModelsComparison) {
    StepInputFeatures features{
        .cadence_hz = 1.8,
        .vertical_accel_p2p = 3.0,
        .vertical_accel_rms = 1.1,
        .terrain_slope_rad = 0.0,
        .step_period_var = 0.01,
        .user_leg_length_m = 0.85
    };

    WeinbergModel weinberg(0.45);
    KimModel kim(0.70);
    ScarlettModel scarlett(0.65);

    const auto w_est = weinberg.estimateStride(features);
    const auto k_est = kim.estimateStride(features);
    const auto s_est = scarlett.estimateStride(features);

    // Tất cả các mô hình cơ sở phải cho giá trị hợp lý trong khoảng [0.5m, 0.9m]
    EXPECT_GT(w_est.mean_length_m, 0.50);
    EXPECT_LT(w_est.mean_length_m, 0.90);

    EXPECT_GT(k_est.mean_length_m, 0.50);
    EXPECT_LT(k_est.mean_length_m, 0.90);

    EXPECT_GT(s_est.mean_length_m, 0.50);
    EXPECT_LT(s_est.mean_length_m, 0.90);
}
