#include <gtest/gtest.h>
#include "survival_core/kinematics/vertical_transport_detector.hpp"

using namespace survival;
using namespace survival::kinematics;

TEST(VerticalTransportTest, DetectEscalatorConstantVelocity) {
    VerticalTransportDetector detector{};

    // Thang cuốn: vz = 0.55 m/s, không có bước chân người (cadence = 0), tốc độ ngang 0.6 m/s
    VerticalTransportScores scores{};
    for (int i = 0; i < 50; ++i) {
        scores = detector.processSample(0.55F, 0.0F, 0.0F, 0.60F, 0.1F, 0.01);
    }

    EXPECT_GT(scores.escalator_score, 0.70F);
    EXPECT_LT(scores.elevator_score, 0.40F);
    EXPECT_LT(scores.cycling_score, 0.30F);
}

TEST(VerticalTransportTest, DetectElevatorHighSpeedVertical) {
    VerticalTransportDetector detector{};

    // Thang máy: vz = 2.5 m/s, không có bước chân người (cadence = 0)
    VerticalTransportScores scores{};
    for (int i = 0; i < 50; ++i) {
        scores = detector.processSample(2.5F, 0.0F, 0.0F, 0.0F, 0.1F, 0.01);
    }

    EXPECT_GT(scores.elevator_score, 0.75F);
    EXPECT_LT(scores.escalator_score, 0.30F);
}

TEST(VerticalTransportTest, DetectCyclingModerateSpeedAndCadence) {
    VerticalTransportDetector detector{};

    // Xe đạp: Nhịp đạp 1.5 Hz, tốc độ ngang 6.0 m/s (21.6 km/h), vz = 0
    VerticalTransportScores scores{};
    for (int i = 0; i < 50; ++i) {
        scores = detector.processSample(0.0F, 0.2F, 1.5F, 6.0F, 0.2F, 0.01);
    }

    EXPECT_GT(scores.cycling_score, 0.70F);
    EXPECT_LT(scores.elevator_score, 0.20F);
}
