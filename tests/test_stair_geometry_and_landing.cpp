#include <gtest/gtest.h>
#include <cmath>
#include "survival_core/kinematics/stair_geometry_estimator.hpp"
#include "survival_core/kinematics/stair_motion_detector.hpp"
#include <cmath>

using namespace survival;
using namespace survival::kinematics;

TEST(StairGeometryTest, AdaptiveRiserEstimationFromBaro) {
    StairGeometryEstimator estimator{};

    EXPECT_NEAR(estimator.getEstimatedRiserM(), 0.165F, 1e-3);

    // Giả lập 10 bước leo cầu thang với tổng độ cao vi sai +1.74m (~0.174m/bước)
    for (int i = 0; i < 10; ++i) {
        estimator.recordStairStep(true, 0.174, true, 0.55);
    }

    const auto state = estimator.getStairState();
    EXPECT_EQ(state.step_count, 10U);
    EXPECT_NEAR(state.estimated_riser_m, 0.174F, 0.015F);
    EXPECT_GT(state.cumulative_vertical_m, 1.5F);
    EXPECT_GT(state.confidence, 0.5F);
    EXPECT_FALSE(state.is_on_landing);
}

TEST(StairGeometryTest, LandingStepTransitions) {
    StairGeometryEstimator estimator{};

    // 5 bước leo cầu thang
    for (int i = 0; i < 5; ++i) {
        estimator.recordStairStep(true, 0.16, true, 0.55);
    }
    EXPECT_FALSE(estimator.getStairState().is_on_landing);

    // 2 bước trên chiếu nghỉ
    estimator.recordLandingStep();
    estimator.recordLandingStep();
    EXPECT_TRUE(estimator.getStairState().is_on_landing);
    EXPECT_EQ(estimator.getStairState().landing_count, 2U);

    // Tiếp tục leo cầu thang tiếp
    estimator.recordStairStep(true, 0.16, true, 0.55);
    EXPECT_FALSE(estimator.getStairState().is_on_landing);
    EXPECT_EQ(estimator.getStairState().step_count, 6U);
}

TEST(StairMotionDetectorTest, RecognizeStairsAscentLongRiseTime) {
    StairMotionDetector detector{};

    // Giả lập sóng leo thang dạng "đẩy sớm" (đã xác minh trên dữ liệu thật: năng
    // lượng đẩy dồn nằm ở ĐẦU chu kỳ bước -> skewness dương; bản cũ có plateau +6
    // sau đỉnh = năng lượng muộn, mã hóa nhầm cực tính descent).
    // Hump đẩy +8 ở 10 mẫu đầu, hố -2.0 tại i=20, rise chậm -2 -> +6 trong 240ms.
    StairDetectorFeatures feat{};
    for (int i = 0; i < 50; ++i) {
        double a_v = 0.0;
        if (i < 10) a_v = 8.0 * std::sin(i / 10.0 * 3.14159265);
        else if (i < 20) a_v = -2.0 * (i - 10) / 10.0;
        else if (i < 44) a_v = -2.0 + (8.0 / 24.0) * (i - 20);
        else a_v = 6.0;

        const bool is_peak = (i == 44);
        const auto f = detector.processSample(a_v, 1.0, 0.01, is_peak);
        if (is_peak) {
            feat = f;
        }
    }

    EXPECT_GE(feat.rise_time_s, 0.20F);
    // [R1] ascent_score = p_huong * p_stair; bước tổng hợp có delta-Z ≈ 0 nên
    // p_stair bị flat-guard cap 0.30 — tiêu chí HƯỚNG nằm ở ascent_likelihood.
    EXPECT_GT(feat.ascent_likelihood, 0.60F);
    EXPECT_GT(feat.ascent_score, feat.descent_score);
}

TEST(StairMotionDetectorTest, RecognizeStairsDescentSharpImpact) {
    StairMotionDetector detector{};

    // Giả lập sóng gia tốc đi xuống cầu thang:
    // Free-fall dip = -4.0 m/s^2 tại i=10, Va đập gót cực gắt +14.0 m/s^2 tại i=15 (5 mẫu = 50ms rise time)
    StairDetectorFeatures feat{};
    for (int i = 0; i < 50; ++i) {
        double a_v = 0.0;
        if (i < 10) a_v = 0.0 - 0.4 * i; // hạ về -4.0 tại i=10
        else if (i <= 15) a_v = -4.0 + (18.0 / 5.0) * (i - 10);
        else a_v = 14.0;

        const bool is_peak = (i == 15);
        const auto f = detector.processSample(a_v, 1.0, 0.01, is_peak);
        if (is_peak) {
            feat = f;
        }
    }

    EXPECT_LE(feat.rise_time_s, 0.08F);
    EXPECT_GE(feat.peak_to_peak_m_s2, 14.0F);
    EXPECT_GT(feat.descent_score, 0.50F);
    EXPECT_LT(feat.ascent_score, 0.30F);
}


// [M7] Riser trung binh chia theo so delta baro HOP LE — baro roi mau giua chang
// khong duoc keo avg xuong (truoc fix: 3 delta 0.17m / 10 buoc = 0.051 -> clamp 0.12)
TEST(StairGeometryTest, RiserAverageUsesOnlyValidBaroDeltas) {
    StairGeometryEstimator est{};

    // 10 buoc chuyen; chi buoc 1, 5, 10 co delta baro hop le (0.17m moi delta)
    for (int step = 1; step <= 10; ++step) {
        const bool has_baro = (step == 1 || step == 5 || step == 10);
        est.recordStairStep(true, has_baro ? 0.17 : 0.0, has_baro, 0.62);
    }

    const float riser = est.getEstimatedRiserM();
    // Truoc fix: avg = 0.51/10 = 0.051 -> clamp ve 0.12 -> EMA keo xuong ~0.13
    EXPECT_NEAR(riser, 0.165F, 0.015F);  // giu gan nguyen gia tri danh dinh
}
