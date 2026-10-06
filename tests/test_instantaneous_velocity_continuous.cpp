#include <gtest/gtest.h>
#include "survival_core/kinematics/instantaneous_velocity_estimator.hpp"
#include <cmath>

using namespace survival;
using namespace survival::kinematics;

TEST(InstantaneousVelocityTest, WalkingOneEuroSmoothnessNoArtificialSineWave) {
    InstantaneousVelocityEstimator estimator{};

    // Bắt đầu đi bộ: cadence = 2.0 Hz, stride = 0.75 m -> target speed = 1.50 m/s
    VelocityEstimateOutput out{};
    for (int i = 0; i < 200; ++i) {
        out = estimator.update(
            MotionActivity::kWalkingFlat,
            0.0,   // Heading = 0 (North)
            0.0,   // yaw_rate = 0
            0.0,   // fwd_accel = 0
            0.75,  // stride
            2.0,   // cadence
            0.165F,// riser
            0.0F,  // baro_rate
            -1.0F, // no gnss
            false, // not stopped
            0.01   // dt = 10ms (100Hz)
        );
    }

    // Sau khi One-Euro hội tụ, vận tốc tiến về 1.50 m/s mượt mà
    EXPECT_NEAR(out.horizontal_speed_m_s, 1.50, 0.05);
    EXPECT_NEAR(out.velocity_enu_m_s.y, 1.50, 0.05); // North
    EXPECT_NEAR(out.velocity_enu_m_s.x, 0.0, 1e-3);  // East
    EXPECT_NEAR(out.vertical_speed_m_s, 0.0, 1e-3);
}

TEST(InstantaneousVelocityTest, StairsAscentVerticalVelocityCalculation) {
    InstantaneousVelocityEstimator estimator{};

    // Leo cầu thang lên: riser = 0.17m, cadence = 2.0 Hz -> vz = +0.34 m/s, v_horiz = 0.28 * 2.0 = 0.56 m/s
    VelocityEstimateOutput out{};
    for (int i = 0; i < 150; ++i) {
        out = estimator.update(
            MotionActivity::kStairsAscent,
            0.0,
            0.0,
            0.0,
            0.75,
            2.0,
            0.17F,
            0.34F,
            -1.0F,
            false,
            0.01
        );
    }

    EXPECT_NEAR(out.horizontal_speed_m_s, 0.56, 0.05);
    EXPECT_NEAR(out.vertical_speed_m_s, 0.34, 0.05);
}

TEST(InstantaneousVelocityTest, StationaryZuptLock) {
    InstantaneousVelocityEstimator estimator{};

    // Đứng yên -> vận tốc triệt tiêu hoàn toàn về 0
    const auto out = estimator.update(
        MotionActivity::kStationary,
        0.0, 0.0, 0.0, 0.75, 0.0, 0.165F, 0.0F, -1.0F, true, 0.01
    );

    EXPECT_DOUBLE_EQ(out.horizontal_speed_m_s, 0.0);
    EXPECT_DOUBLE_EQ(out.vertical_speed_m_s, 0.0);
    EXPECT_DOUBLE_EQ(out.velocity_enu_m_s.x, 0.0);
    EXPECT_DOUBLE_EQ(out.velocity_enu_m_s.y, 0.0);
}


// [B4] Nganh cycling dung GNSS Doppler khi co (thay 3.5*cadence gia lap)
TEST(InstantaneousVelocityTest, CyclingUsesGnssDopplerSpeed) {
    InstantaneousVelocityEstimator estimator{};
    estimator.reset();

    VelocityEstimateOutput out{};
    for (int i = 0; i < 200; ++i) {
        out = estimator.update(
            MotionActivity::kCycling,
            0.0,      // heading
            0.0,      // yaw rate
            0.0,      // fwd accel
            0.75,     // stride (khong dung cho cycling)
            1.5,      // cadence — 3.5*1.5 = 5.25 neu khong co GNSS
            0.0F,     // riser
            0.0F,     // baro vz
            6.0F,     // gnss_speed_m_s [B4]
            false,
            0.01);
    }
    EXPECT_NEAR(out.horizontal_speed_m_s, 6.0, 0.3);
}
