// ============================================================================
// [A2] GNSS-aided stride scale (Weiberg K) tests
// ============================================================================
#include <gtest/gtest.h>
#include "survival_core/kinematics/stride_scale_estimator.hpp"
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/dem_map.hpp"

#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;
using survival::kinematics::StrideScaleEstimator;

// ----------------------------------------------------------------------------
// Co che EMA thuan: day innovation ratio 1.2 lap lai phai keo scale ve 1.2
// ----------------------------------------------------------------------------
TEST(StrideScaleEstimator, ConvergesToRepeatedRatio) {
    StrideScaleEstimator est{};
    for (int i = 0; i < 10; ++i) {
        est.update(1.2);
    }
    EXPECT_GT(est.scale(), 1.15);
    EXPECT_LT(est.scale(), 1.25);
    EXPECT_EQ(est.updateCount(), 10U);
}

TEST(StrideScaleEstimator, RejectsOutlierRatios) {
    StrideScaleEstimator est{};
    est.update(1.2);
    const double before = est.scale();
    // Ratio ngoai [0.5, 2.0] = outlier (GPS glitch / stair-override corrupt) - bo luot
    EXPECT_EQ(est.update(5.0), before);
    EXPECT_EQ(est.update(0.1), before);
    EXPECT_EQ(est.updateCount(), 1U);
}

TEST(StrideScaleEstimator, ConvergesDownwardToo) {
    StrideScaleEstimator est{};
    for (int i = 0; i < 10; ++i) {
        est.update(0.85);  // sai mo hình dai hon that 15%
    }
    EXPECT_GT(est.scale(), 0.80);
    EXPECT_LT(est.scale(), 0.90);
}

TEST(StrideScaleEstimator, ClampedToBounds) {
    StrideScaleEstimator est{};
    for (int i = 0; i < 50; ++i) {
        est.update(1.9);  // lien tuc keo len - phai kep tai max 1.4
    }
    EXPECT_NEAR(est.scale(), 1.40, 0.001);
}

// ----------------------------------------------------------------------------
// End-to-end nhe: pipeline co anchor GNSS phai chay qua duong hieu chuan
// ma khong crash va scale luon nam trong clamp
// ----------------------------------------------------------------------------
TEST(GnssStrideCalibration, PipelineScaleStaysBoundedWithAnchors) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(2000.0, 2000.0, 10.0, 1000.0, 1000.0, 2000.0, 1000.0, 500.0));
    pipeline.initialize(EastingNorthingAlt{1000.0, 750.0, 1500.0}, dem);
    uint64_t t_ns = 1000000000ULL;

    double true_e = 1000.0, true_n = 750.0;
    for (int cycle = 0; cycle < 5; ++cycle) {
        for (int step = 0; step < 10; ++step) {
            constexpr size_t kSamples = 55;
            for (size_t i = 0; i < kSamples; ++i) {
                const double phase = static_cast<double>(i) / static_cast<double>(kSamples);
                t_ns += 10000000ULL;
                SensorFrame frame{};
                frame.timestamp_ns = t_ns;
                frame.has_imu = true;
                frame.has_baro = true;
                frame.has_mag = true;
                const double a_dyn = (phase < 0.50)
                    ? 18.0 * std::sin(phase / 0.50 * 3.14159265358979323846)
                    : -6.0 * std::sin((phase - 0.50) / 0.50 * 3.14159265358979323846);
                frame.accel_m_s2 = {0.0F, 0.0F, static_cast<float>(9.81 + a_dyn)};
                frame.pressure_hpa = 1013.25F;
                frame.mag_uT = {0.0F, 20.0F, -40.0F};
                (void)pipeline.processSensorFrame(frame);
            }
            true_n += 0.9;
        }
        pipeline.updateAnchorPosition(EastingNorthingAlt{true_e, true_n, 1500.0}, 2.0);
        EXPECT_GE(pipeline.getStrideScale(), 0.70);
        EXPECT_LE(pipeline.getStrideScale(), 1.40);
    }
}
