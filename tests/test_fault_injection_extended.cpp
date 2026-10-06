#include <gtest/gtest.h>
#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/map_manager.hpp"
#include "survival_core/integrity/integrity_monitor.hpp"
#include "survival_core/safety/resource_governor.hpp"
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;
using namespace survival::integrity;
using namespace survival::safety;

// ----------------------------------------------------------------------------
// SCENARIO 1: GPS Cutoff Handoff (Safety Invariant 1: No Teleportation)
// ----------------------------------------------------------------------------
TEST(FaultInjectionExtendedTest, Scenario1_GpsCutoffZeroTeleportation) {
    LocalFrame frame{
        .origin_lat_rad = 11.940419 * kDegToRad,
        .origin_lon_rad = 108.458313 * kDegToRad,
        .origin_alt_m = 1500.0
    };
    MapManager manager(frame);

    // Bơm 5 mẫu GPS hợp lệ tại (500m East, 300m North)
    for (int i = 0; i < 5; ++i) {
        GeodeticCoord geo = frame.enuToGeodetic(EastingNorthingAlt{500.0 + i * 2.0, 300.0 + i * 2.0, 1500.0});
        GpsSample sample{
            .lat_rad = geo.latitude_deg * kDegToRad,
            .lon_rad = geo.longitude_deg * kDegToRad,
            .alt_m = 1500.0,
            .std_pos_m = 3.0,
            .hdop = 1.0,
            .timestamp_ns = 1000000000ULL + i * 1000000000ULL,
            .is_valid = true
        };
        manager.handleGpsFix(sample, 1500.0);
    }

    // Đột ngột mất tín hiệu GPS tại t = 6s
    const auto handoff = manager.handleGpsCutoff(6000000000ULL);
    ASSERT_TRUE(handoff.is_valid);

    // Kiểm tra độ dịch chuyển so với mẫu GPS cuối cùng
    const double de = handoff.anchor_pos.easting_m - 508.0;
    const double dn = handoff.anchor_pos.northing_m - 308.0;
    const double dist_error = std::sqrt(de * de + dn * dn);

    // Không được nhảy về (0, 0) hoặc dịch chuyển > 3*sigma (9m)
    EXPECT_LT(dist_error, 9.0);
    EXPECT_GT(handoff.anchor_pos.easting_m, 400.0);
    EXPECT_GT(handoff.anchor_pos.northing_m, 200.0);
}

// ----------------------------------------------------------------------------
// SCENARIO 2: GPS 500m Multipath Spoofing Jump (Temporal Consistency Rejection)
// ----------------------------------------------------------------------------
TEST(FaultInjectionExtendedTest, Scenario2_Gps500mMultipathJumpRejection) {
    LocalFrame frame{
        .origin_lat_rad = 11.940419 * kDegToRad,
        .origin_lon_rad = 108.458313 * kDegToRad,
        .origin_alt_m = 1500.0
    };
    MapManager manager(frame);

    // Điểm GPS chuẩn tại t = 1.0s
    GeodeticCoord geo1 = frame.enuToGeodetic(EastingNorthingAlt{100.0, 100.0, 1500.0});
    GpsSample fix1{
        .lat_rad = geo1.latitude_deg * kDegToRad,
        .lon_rad = geo1.longitude_deg * kDegToRad,
        .alt_m = 1500.0,
        .std_pos_m = 3.0,
        .hdop = 1.0,
        .timestamp_ns = 1000000000ULL,
        .is_valid = true
    };
    manager.handleGpsFix(fix1, 1500.0);

    // Cú nhảy 500m chỉ sau 0.5s với độ bất định cao (Spoofing / Multipath)
    GeodeticCoord geo2 = frame.enuToGeodetic(EastingNorthingAlt{600.0, 100.0, 1500.0});
    GpsSample fix_jump{
        .lat_rad = geo2.latitude_deg * kDegToRad,
        .lon_rad = geo2.longitude_deg * kDegToRad,
        .alt_m = 1500.0,
        .std_pos_m = 18.0, // Nhiễu lớn
        .hdop = 3.5,
        .timestamp_ns = 1500000000ULL,
        .is_valid = true
    };
    manager.handleGpsFix(fix_jump, 1500.0);

    // LastKnownGoodFix phải giữ nguyên vị trí cũ (100, 100) chứ không nhận cú nhảy 500m
    const auto& last_good = manager.getLastKnownGoodFix();
    EXPECT_NEAR(last_good.easting_m, 100.0, 1.0);
    EXPECT_NEAR(last_good.northing_m, 100.0, 1.0);
}

// ----------------------------------------------------------------------------
// SCENARIO 3: Extreme 150 uT Magnetic Storm / Soft Iron Anomaly (Invariant 4)
// ----------------------------------------------------------------------------
TEST(FaultInjectionExtendedTest, Scenario3_Extreme150uTMagneticStorm) {
    EskfConfig cfg{};
    cfg.mag_max_nis = 9.0;
    Eskf3D eskf(cfg);

    // Cập nhật gia tốc tĩnh để định vị phương thẳng đứng
    eskf.updateAccelerometer(Vector3f{0.0F, 0.0F, 9.80665F});

    // Mẫu từ trường chuẩn Trái Đất ~ 45 uT (North = 30 uT, Down = -35 uT)
    const bool ok_norm = eskf.updateMagnetometer(Vector3f{0.0F, 30.0F, -35.0F});
    EXPECT_TRUE(ok_norm);
    EXPECT_LT(eskf.getLatestMagNis(), 9.0);

    // Bơm mẫu bão từ trường cực đoan 150 uT
    const bool ok_storm = eskf.updateMagnetometer(Vector3f{120.0F, 80.0F, 50.0F});
    EXPECT_FALSE(ok_storm); // Bắt buộc bị Gate từ chối hoàn toàn
    EXPECT_LT(eskf.getMagHealth(), 1.0F); // Điểm sức khỏe từ kế phải giảm
}

// ----------------------------------------------------------------------------
// SCENARIO 4: Frozen Barometer (Safety Invariant 10: No NaN / Inf)
// ----------------------------------------------------------------------------
TEST(FaultInjectionExtendedTest, Scenario4_FrozenBarometerNoNanInf) {
    MultiModalFusionPipeline pipeline{};
    pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 1000.0}, DemMap{});

    // Bơm 200 mẫu cảm biến liên tục với áp suất đóng băng cứng ở 850.0 hPa
    for (uint64_t i = 0; i < 200; ++i) {
        SensorFrame frame{};
        frame.timestamp_ns = 1000000000ULL + i * 10000000ULL; // 100 Hz
        frame.accel_m_s2 = {0.0F, 0.0F, 9.80665F + static_cast<float>(std::sin(static_cast<double>(i) * 0.2) * 2.0)};
        frame.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        frame.mag_uT = {0.0F, 30.0F, -35.0F};
        frame.pressure_hpa = 850.0F; // Đóng băng
        frame.has_imu = true;
        frame.has_mag = true;
        frame.has_baro = true;

        auto state_opt = pipeline.processSensorFrame(frame);
        if (state_opt) {
            EXPECT_FALSE(std::isnan(state_opt->clusters[0].center_altitude_m));
            EXPECT_FALSE(std::isinf(state_opt->clusters[0].center_altitude_m));
            EXPECT_FALSE(std::isnan(state_opt->global_navigation_score));
        }
    }
}

// ----------------------------------------------------------------------------
// SCENARIO 5: Timestamp Discontinuity & Jitter (Safety Invariant 9)
// ----------------------------------------------------------------------------
TEST(FaultInjectionExtendedTest, Scenario5_TimestampJitterProtection) {
    Eskf3D eskf{};

    // Bơm mẫu bình thường ở t = 1.0s
    eskf.predict(Vector3f{0.01F, 0.0F, 0.0F}, 0.01);

    // Bơm bước thời gian âm hoặc nhảy vọt (dt = -0.5s hoặc dt = 10.0s)
    eskf.predict(Vector3f{0.01F, 0.0F, 0.0F}, -0.5); // Bỏ qua
    eskf.predict(Vector3f{0.01F, 0.0F, 0.0F}, 10.0); // Bỏ qua

    const auto att = eskf.getAttitudeEstimate();
    EXPECT_FALSE(std::isnan(att.euler.roll_rad));
    EXPECT_FALSE(std::isnan(att.euler.pitch_rad));
    EXPECT_FALSE(std::isnan(att.euler.yaw_rad));
}
