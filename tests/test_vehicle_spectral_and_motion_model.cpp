#include <gtest/gtest.h>
#include "survival_core/kinematics/vehicle_motion_detector.hpp"
#include "survival_core/kinematics/vehicle_motion_model.hpp"
#include <cmath>

using namespace survival;
using namespace survival::kinematics;

TEST(VehicleSpectralTest, HighFrequencyMotorVibrationDetection) {
    VehicleMotionDetector detector{};

    // Tạo tín hiệu rung động động cơ xe 25 Hz (dt = 0.01s)
    VehicleFeatures feat{};
    for (int i = 0; i < 150; ++i) {
        const double t = i * 0.01;
        const float vibration_25hz = 1.2F * std::sin(2.0 * kPi * 25.0 * t);
        const Vector3f accel{0.1F, 0.1F, static_cast<float>(kGravityStandard) + vibration_25hz};
        feat = detector.processSample(accel, -1.0F, 0.0F, 0.01);
    }

    EXPECT_GT(feat.energy_high_12_40hz, 0.05F);
    EXPECT_GT(feat.hf_ratio, 1.0F);
    EXPECT_GT(feat.vehicle_score, 0.40F);
}

TEST(VehicleSpectralTest, GnssSpeedSuppressionAndStopState) {
    VehicleMotionDetector detector{};

    // Tín hiệu xe chạy nhanh 15 m/s (54 km/h)
    Vector3f accel{0.0F, 0.0F, static_cast<float>(kGravityStandard)};
    auto feat = detector.processSample(accel, 15.0F, 0.0F, 0.01);
    EXPECT_GT(feat.vehicle_score, 0.80F);
    EXPECT_FALSE(feat.is_stopped);

    // Tín hiệu xe dừng đèn đỏ (tốc độ GPS = 0, rung động nhẹ)
    for (int i = 0; i < 200; ++i) {
        const float vib = 0.3F * std::sin(2.0 * kPi * 20.0 * (i * 0.01));
        Vector3f acc{0.0F, 0.0F, static_cast<float>(kGravityStandard) + vib};
        feat = detector.processSample(acc, 0.0F, 0.0F, 0.01);
    }
    EXPECT_TRUE(feat.is_stopped);
}

TEST(VehicleMotionModelTest, KinematicUnicycleDeadReckoningWithDrag) {
    VehicleMotionModel model{};

    // 1. Tăng tốc trong 5 giây với gia tốc 2.0 m/s^2
    for (int i = 0; i < 500; ++i) {
        model.update(0.0, 0.0, 2.0, -1.0F, false, 0.01);
    }
    const double speed_after_accel = model.getCurrentSpeed();
    EXPECT_NEAR(speed_after_accel, 8.8, 1.5);

    // 2. Chạy trớn không gia tốc trong 10 giây -> tốc độ giảm dần do drag nhưng không bị nhảy âm
    for (int i = 0; i < 1000; ++i) {
        model.update(0.0, 0.0, 0.0, -1.0F, false, 0.01);
    }
    const double speed_after_coast = model.getCurrentSpeed();
    EXPECT_LT(speed_after_coast, speed_after_accel);
    EXPECT_GT(speed_after_coast, 0.0);

    // 3. Phanh dừng đỗ
    const auto stopped_state = model.update(0.0, 0.0, 0.0, -1.0F, true, 0.01);
    EXPECT_DOUBLE_EQ(stopped_state.speed_m_s, 0.0);
    EXPECT_DOUBLE_EQ(stopped_state.velocity_enu_m_s.x, 0.0);
    EXPECT_DOUBLE_EQ(stopped_state.velocity_enu_m_s.y, 0.0);
}
