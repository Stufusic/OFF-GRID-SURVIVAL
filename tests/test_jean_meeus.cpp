#include <gtest/gtest.h>
#include "survival_core/environment/jean_meeus_solver.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

using namespace survival;
using namespace survival::environment;
using namespace survival::math;

TEST(JeanMeeusTest, JulianDayKnownVectorJ2000) {
    // 2000-01-01 12:00:00 UTC phải tương ứng chính xác JD 2451545.0
    const double jd = JeanMeeusSolver::computeJulianDay(2000, 1, 1, 12.0);
    EXPECT_DOUBLE_EQ(jd, 2451545.0);
}

TEST(JeanMeeusTest, SpringEquinoxSolarNoonGreenwich) {
    // Thời điểm Xuân Phân 2026: 2026-03-20 12:00:00 UTC
    // Unix timestamp: 1774008000 giây (tương đương 1.774008e18 ns)
    constexpr uint64_t timestamp_ns = 1774008000ULL * 1000000000ULL;
    const double lat_greenwich = degToRad(51.48); // Vĩ độ Greenwich
    const double lon_greenwich = degToRad(0.0);   // Kinh độ 0

    const auto solar = JeanMeeusSolver::computeSolarPosition(timestamp_ns, lat_greenwich, lon_greenwich);

    // 1. Ngày Xuân phân: Xích vĩ Mặt Trời (Declination) xấp xỉ 0 độ
    EXPECT_NEAR(radToDeg(solar.declination_rad), 0.0, 1.0);

    // 2. Vào buổi trưa ở bán cầu Bắc: Góc phương vị Mặt Trời xấp xỉ 180 độ (hướng Nam)
    EXPECT_NEAR(radToDeg(solar.azimuth_rad), 180.0, 5.0);

    // 3. Ban ngày (Elevation > 0)
    EXPECT_TRUE(solar.is_daylight);
    EXPECT_GT(solar.elevation_rad, 0.0);
}

TEST(JeanMeeusTest, SolarAzimuthLikelihoodEvaluation) {
    constexpr double expected_az = degToRad(180.0);
    constexpr double sigma = degToRad(3.0);

    // Trùng khớp hoàn toàn -> Likelihood = 1.0
    const double l_perfect = JeanMeeusSolver::computeSolarAzimuthLikelihood(expected_az, expected_az, sigma);
    EXPECT_NEAR(l_perfect, 1.0, 1e-6);

    // Lệch 3 sigma (~9 độ) -> Likelihood = exp(-0.5 * 3^2) = exp(-4.5) ~ 0.0111
    const double l_3sigma = JeanMeeusSolver::computeSolarAzimuthLikelihood(expected_az + 3.0 * sigma, expected_az, sigma);
    EXPECT_NEAR(l_3sigma, std::exp(-4.5), 0.001);
}
