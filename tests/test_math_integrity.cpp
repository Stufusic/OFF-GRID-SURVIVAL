#include <gtest/gtest.h>
#include <cmath>
#include <string_view>
#include "survival_core/types.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_core/math/crc32.hpp"
#include "survival_core/math/statistics.hpp"

using namespace survival;
using namespace survival::math;

TEST(AngleUtilsTest, NormalizeAngleRadBoundaries) {
    EXPECT_NEAR(normalizeAngleRad(0.0), 0.0, 1e-9);
    EXPECT_NEAR(normalizeAngleRad(kPi), kPi, 1e-9);
    EXPECT_NEAR(normalizeAngleRad(-kPi), -kPi, 1e-9);
    EXPECT_NEAR(normalizeAngleRad(3.0 * kPi), kPi, 1e-9);
    EXPECT_NEAR(normalizeAngleRad(-3.0 * kPi), -kPi, 1e-9);
    EXPECT_NEAR(normalizeAngleRad(kTwoPi + 0.5), 0.5, 1e-9);
    EXPECT_NEAR(normalizeAngleRad(-kTwoPi - 0.5), -0.5, 1e-9);
}

TEST(AngleUtilsTest, NormalizeHeadingRad) {
    EXPECT_NEAR(normalizeHeadingRad(0.0), 0.0, 1e-9);
    EXPECT_NEAR(normalizeHeadingRad(kHalfPi), kHalfPi, 1e-9);
    EXPECT_NEAR(normalizeHeadingRad(kPi), kPi, 1e-9);
    EXPECT_NEAR(normalizeHeadingRad(-kHalfPi), 3.0 * kHalfPi, 1e-9);
    EXPECT_NEAR(normalizeHeadingRad(-0.001), kTwoPi - 0.001, 1e-9);
    EXPECT_NEAR(normalizeHeadingRad(kTwoPi + 0.1), 0.1, 1e-9);
}

TEST(AngleUtilsTest, DegreeConversions) {
    EXPECT_NEAR(degToRad(180.0), kPi, 1e-9);
    EXPECT_NEAR(radToDeg(kPi), 180.0, 1e-9);
    EXPECT_NEAR(normalizeAngleDeg(370.0), 10.0, 1e-9);
    EXPECT_NEAR(normalizeAngleDeg(-190.0), 170.0, 1e-9);
    EXPECT_NEAR(normalizeHeadingDeg(-90.0), 270.0, 1e-9);
}

TEST(AngleUtilsTest, AngleDifference) {
    EXPECT_NEAR(angleDifferenceRad(0.1, -0.1), 0.2, 1e-9);
    EXPECT_NEAR(angleDifferenceRad(0.1, kTwoPi - 0.1), 0.2, 1e-9);
    EXPECT_NEAR(angleDifferenceRad(kPi - 0.1, -kPi + 0.1), -0.2, 1e-9);
}

TEST(Crc32Test, StandardKnownVector) {
    // Chuỗi kiểm thử chuẩn: "123456789" có CRC-32 = 0xCBF43926
    constexpr std::string_view kInput = "123456789";
    const uint32_t crc = Crc32::calculate(kInput.data(), kInput.size());
    EXPECT_EQ(crc, 0xCBF43926U);
}

TEST(StatisticsTest, UncertaintyEllipseIsotropic) {
    // Phân phối đẳng hướng: var_e = 100, var_n = 100, cov = 0
    const auto ellipse = computeUncertaintyEllipse(100.0, 100.0, 0.0, kChiSquare2D_95);
    const double expected_radius = std::sqrt(kChiSquare2D_95 * 100.0); // ≈ 24.477 m
    EXPECT_NEAR(ellipse.semi_major_m, expected_radius, 1e-4);
    EXPECT_NEAR(ellipse.semi_minor_m, expected_radius, 1e-4);
    EXPECT_NEAR(ellipse.area_m2, kPi * expected_radius * expected_radius, 1e-2);
}

TEST(StatisticsTest, UncertaintyEllipse1DDegeneracyProtection) {
    // Suy biến 1D: var_e = 100, var_n = 0, cov = 0
    const auto ellipse = computeUncertaintyEllipse(100.0, 0.0, 0.0, kChiSquare2D_95);
    EXPECT_GT(ellipse.semi_major_m, 20.0);
    // Bán trục nhỏ không được bằng 0 hoặc âm, mà phải bị chặn ở sqrt(kChiSquare2D_95 * 1e-6)
    EXPECT_GT(ellipse.semi_minor_m, 0.0);
    EXPECT_FALSE(std::isnan(ellipse.semi_minor_m));
    EXPECT_FALSE(std::isinf(ellipse.semi_minor_m));
    EXPECT_GT(ellipse.area_m2, 0.0);
}
