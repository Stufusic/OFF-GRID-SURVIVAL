#include <gtest/gtest.h>
#include "survival_harness/failure_injector.hpp"
#include <cmath>

using namespace survival;
using namespace survival::harness;

TEST(FailureInjectorTest, M1MagneticSpikeTimeWindow) {
    FailureConfig config{};
    config.scenario = ScenarioType::kM1_MagneticSpike;
    config.mag_spike_start_ns = 5ULL * 1000000000ULL;  // 5s
    config.mag_spike_end_ns = 15ULL * 1000000000ULL;   // 15s
    config.mag_spike_magnitude_uT = 45.0;

    FailureInjector injector(config);

    // 1. Trước thời điểm spike (t = 2s) -> Không bị tiêm nhiễu
    SensorFrame frame_before{};
    frame_before.timestamp_ns = 2ULL * 1000000000ULL;
    frame_before.mag_uT = {0.0F, 10.0F, 0.0F};
    injector.injectSensorFrame(frame_before);
    EXPECT_FLOAT_EQ(frame_before.mag_uT[1], 10.0F);

    // 2. Trong thời điểm spike (t = 10s) -> Được cộng thêm +45 uT
    SensorFrame frame_inside{};
    frame_inside.timestamp_ns = 10ULL * 1000000000ULL;
    frame_inside.mag_uT = {0.0F, 10.0F, 0.0F};
    injector.injectSensorFrame(frame_inside);
    EXPECT_FLOAT_EQ(frame_inside.mag_uT[1], 55.0F);

    // 3. Sau thời điểm spike (t = 20s) -> Hết nhiễu
    SensorFrame frame_after{};
    frame_after.timestamp_ns = 20ULL * 1000000000ULL;
    frame_after.mag_uT = {0.0F, 10.0F, 0.0F};
    injector.injectSensorFrame(frame_after);
    EXPECT_FLOAT_EQ(frame_after.mag_uT[1], 10.0F);
}

TEST(FailureInjectorTest, M2BarometricLinearDrift) {
    FailureConfig config{};
    config.scenario = ScenarioType::kM2_BarometricDrift;
    config.baro_drift_start_ns = 0ULL;
    config.baro_drift_hpa_per_hour = 2.0; // 2.0 hPa / giờ

    FailureInjector injector(config);

    // Sau 1800 giây (0.5 giờ), áp suất phải giảm đúng 1.0 hPa
    SensorFrame frame{};
    frame.timestamp_ns = 1800ULL * 1000000000ULL;
    frame.pressure_hpa = 1013.25F;

    injector.injectSensorFrame(frame);
    EXPECT_NEAR(frame.pressure_hpa, 1012.25F, 0.01F);
}

TEST(FailureInjectorTest, M3CelestialOutageRejection) {
    FailureConfig config{};
    config.scenario = ScenarioType::kM3_CelestialOutage;
    config.celestial_outage_start_ns = 10ULL * 1000000000ULL;
    config.celestial_outage_end_ns = 30ULL * 1000000000ULL;

    FailureInjector injector(config);

    SensorFrame frame{};
    frame.timestamp_ns = 15ULL * 1000000000ULL;
    frame.has_optical_sun = true;
    frame.optical_sun_azimuth_rad = 3.14F;
    frame.optical_confidence = 0.95F;

    injector.injectSensorFrame(frame);
    EXPECT_FALSE(frame.has_optical_sun);
    EXPECT_FLOAT_EQ(frame.optical_sun_azimuth_rad, -1.0F);
    EXPECT_FLOAT_EQ(frame.optical_confidence, 0.0F);
}

TEST(FailureInjectorTest, M4PdrNoiseCorruption) {
    FailureConfig config{};
    config.scenario = ScenarioType::kM4_PdrNoiseInflation;
    config.stride_corruption_std = 0.20;
    config.heading_corruption_std_rad = 0.15;
    config.environment_seed = 42;

    FailureInjector injector(config);

    StepEvent step{};
    step.stride_mean_m = 0.75;
    step.heading_rad = 0.0;
    step.stride_variance_m2 = 0.0016;
    step.heading_variance_rad2 = 0.01;

    injector.injectStepEvent(step);

    // Bị biến thiên và phương sai được phóng đại
    EXPECT_NE(step.stride_mean_m, 0.75);
    EXPECT_GT(step.stride_variance_m2, 0.0016);
    EXPECT_GT(step.heading_variance_rad2, 0.01);
}


// [M12] M4 phai thuc su tiem nhieu IMU trong injectSensorFrame — truoc day M4
// chi co injectStepEvent (khong ai goi trong benchmark) => M4 trung dung M0
TEST(FailureInjectorM12, M4InjectsImuNoiseOnSensorFrame) {
    FailureConfig cfg{};
    cfg.scenario = ScenarioType::kM4_PdrNoiseInflation;
    cfg.environment_seed = 42;
    FailureInjector injector{cfg};

    SensorFrame base{};
    base.timestamp_ns = 1000000000ULL;
    base.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    base.gyro_rad_s = {0.0F, 0.0F, 0.0F};
    base.has_imu = true;

    int accel_changed = 0;
    int gyro_changed = 0;
    constexpr int kTrials = 100;
    for (int i = 0; i < kTrials; ++i) {
        SensorFrame f = base;
        injector.injectSensorFrame(f);
        if (std::abs(f.accel_m_s2[0] - base.accel_m_s2[0]) > 1e-6 ||
            std::abs(f.accel_m_s2[2] - base.accel_m_s2[2]) > 1e-6) {
            accel_changed++;
        }
        if (std::abs(f.gyro_rad_s[2] - base.gyro_rad_s[2]) > 1e-6) {
            gyro_changed++;
        }
        // Nhieu phai co do lon hop ly (3-sigma: |accel| < 6, |gyro| < 0.6)
        EXPECT_LT(std::abs(f.accel_m_s2[0]), 6.0F);
        EXPECT_LT(std::abs(f.gyro_rad_s[2]), 0.6F);
    }
    EXPECT_GT(accel_changed, kTrials / 2);
    EXPECT_GT(gyro_changed, kTrials / 2);

    // M0 KHONG duoc tiem gi
    FailureConfig cfg0{};
    cfg0.scenario = ScenarioType::kM0_Nominal;
    FailureInjector injector0{cfg0};
    SensorFrame f0 = base;
    injector0.injectSensorFrame(f0);
    EXPECT_EQ(f0.accel_m_s2[2], base.accel_m_s2[2]);
}
