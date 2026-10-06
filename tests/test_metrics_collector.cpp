#include <gtest/gtest.h>
#include "survival_harness/metrics_collector.hpp"
#include <random>
#include <cmath>

using namespace survival;
using namespace survival::harness;

TEST(MetricsCollectorTest, KnownGaussianErrorDistribution) {
    MetricsCollector collector;

    // Sinh 10,000 mẫu sai số Gauss đẳng hướng sigma = 10m
    constexpr double kSigma = 10.0;
    constexpr size_t kNumSamples = 10000;

    std::mt19937_64 rng(12345);
    std::normal_distribution<double> dist(0.0, kSigma);

    for (size_t i = 0; i < kNumSamples; ++i) {
        const double err_e = dist(rng);
        const double err_n = dist(rng);

        const EastingNorthingAlt truth{100.0, 200.0, 50.0};
        const EastingNorthingAlt est{100.0 + err_e, 200.0 + err_n, 50.0};

        collector.recordEstimate(
            static_cast<uint64_t>(i * 1000000000ULL),
            est,
            truth,
            kSigma * kSigma, // var_e = 100
            kSigma * kSigma, // var_n = 100
            0.0              // cov_en = 0
        );
    }

    const auto report = collector.computeReport();

    // 1. Kiểm tra số lượng mẫu
    EXPECT_EQ(report.sample_count, kNumSamples);

    // 2. CE50 lý thuyết: sqrt(1.3863) * 10 ≈ 11.77 m
    EXPECT_NEAR(report.ce50_m, 11.77, 0.5);

    // 3. CE95 lý thuyết: sqrt(5.9915) * 10 ≈ 24.48 m
    EXPECT_NEAR(report.ce95_m, 24.48, 0.8);

    // 4. RMSE 2D lý thuyết: sqrt(2 * sigma^2) = sqrt(200) ≈ 14.14 m
    EXPECT_NEAR(report.rmse_2d_m, 14.14, 0.5);

    // 5. Mean NEES 2D lý thuyết: Kỳ vọng = 2.0 (bậc tự do k=2)
    EXPECT_NEAR(report.mean_nees_2d, 2.0, 0.1);

    // 6. Tỷ lệ chứa elip 95% lý thuyết: 95.0%
    EXPECT_NEAR(report.empirical_containment_95, 95.0, 1.5);
}

TEST(MetricsCollectorTest, InnovationNIS) {
    MetricsCollector collector;

    // Sinh 1000 mẫu innovation chuẩn hóa (N(0, sigma^2) với sigma = 2.0)
    std::mt19937_64 rng(54321);
    std::normal_distribution<double> dist(0.0, 2.0);

    for (size_t i = 0; i < 1000; ++i) {
        const double innov = dist(rng);
        collector.recordInnovation(static_cast<uint64_t>(i * 1000000ULL), innov, 4.0); // var = 4.0
    }

    const auto report = collector.computeReport();

    // NIS kỳ vọng = 1.0 (k=1 bậc tự do)
    EXPECT_NEAR(report.mean_nis, 1.0, 0.1);
}

// ============================================================================
// [H3] Elip 95% <-> Hiệp phương sai: đơn vị NEES phải là M^2, không phải M
// ============================================================================
#include "survival_core/math/statistics.hpp"

using survival::math::computeUncertaintyEllipse;
using survival::math::ellipseToCov2D;
using survival::math::kChiSquare2D_95;

TEST(NeesUnits, EllipseToCovRoundtrip) {
    const std::array<double, 4> cov_orig{4.0, 1.2, 1.2, 2.0}; // [var_e, cov_en, cov_en, var_n]
    const auto ell = computeUncertaintyEllipse(cov_orig[0], cov_orig[3], cov_orig[1]);
    const auto cov_back = ellipseToCov2D(ell.semi_major_m, ell.semi_minor_m, ell.orientation_rad);

    EXPECT_NEAR(cov_back[0], cov_orig[0], 1e-9);
    EXPECT_NEAR(cov_back[1], cov_orig[1], 1e-9);
    EXPECT_NEAR(cov_back[3], cov_orig[3], 1e-9);
}

TEST(NeesUnits, KnownErrorGivesKnownNees) {
    MetricsCollector metrics;
    const EastingNorthingAlt gt{100.0, 200.0, 50.0};
    for (int i = 0; i < 100; ++i) {
        // Lỗi vector (1, 0), P = I -> NEES = e^T P^-1 e = 1.0
        const EastingNorthingAlt est{101.0, 200.0, 50.0};
        metrics.recordEstimate(static_cast<uint64_t>(i) * 1000000ULL, est, gt, 1.0, 1.0, 0.0);
    }
    const auto report = metrics.computeReport();
    EXPECT_NEAR(report.mean_nees_2d, 1.0, 1e-9);
}

TEST(NeesUnits, ConsistentFilterContainmentNear95Pct) {
    MetricsCollector metrics;
    // P = [[2.0, 0.6], [0.6, 1.0]] — sinh lỗi ~ N(0, P) qua Cholesky
    const double var_e = 2.0, var_n = 1.0, cov_en = 0.6;
    const double l11 = std::sqrt(var_e);
    const double l21 = cov_en / l11;
    const double l22 = std::sqrt(std::max(1e-9, var_n - l21 * l21));

    std::mt19937 rng(20260906);
    std::normal_distribution<double> gauss(0.0, 1.0);
    const EastingNorthingAlt gt{0.0, 0.0, 0.0};
    for (int i = 0; i < 10000; ++i) {
        const double z1 = gauss(rng), z2 = gauss(rng);
        const EastingNorthingAlt est{l11 * z1, l21 * z1 + l22 * z2, 0.0};
        metrics.recordEstimate(static_cast<uint64_t>(i) * 1000000ULL, est, gt, var_e, var_n, cov_en);
    }
    const auto report = metrics.computeReport();
    // Trước fix [H3] (truyền a*0.5 làm phương sai): containment bão hòa ~100% => mù inconsistency
    // Lưu ý: empirical_containment_95 biểu thị theo PHẦN TRĂM (94.92 = 94.92%)
    EXPECT_GE(report.empirical_containment_95, 93.0);
    EXPECT_LE(report.empirical_containment_95, 97.0);
    // NEES nhất quán 2-DOF: kỳ vọng 2.0
    EXPECT_GE(report.mean_nees_2d, 1.8);
    EXPECT_LE(report.mean_nees_2d, 2.2);
}
