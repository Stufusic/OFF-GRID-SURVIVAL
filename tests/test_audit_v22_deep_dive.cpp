#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/kinematics/motion_activity_classifier.hpp"
#include "survival_core/integrity/integrity_monitor.hpp"
#include "survival_core/c_api/survival_c_api.h"
#include "survival_platform/platform_factory.hpp"
#include <cmath>
#include <limits>

using namespace survival;
using namespace survival::estimators;
using namespace survival::kinematics;
using namespace survival::integrity;

// 1. Kiểm tra lỗi Particle Filter bị đóng băng vị trí khi đi xe
TEST(AuditDeepDiveTest, VehicleMotionParticleCloudFrozenBug) {
    MultiModalFusionPipeline pipeline;
    environment::DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(5000.0, 5000.0, 10.0, 100.0, 500.0));
    pipeline.initialize(EastingNorthingAlt{100.0, 100.0, 200.0}, dem);

    // Mô phỏng xe chạy về phía Đông (East) với vận tốc 15 m/s trong 10 giây (1000 mẫu @ 100Hz)
    // Tín hiệu rung động động cơ 25Hz để VehicleDetector nhận diện kVehicleWheeled
    NavigationUncertaintyState last_state{};
    for (uint64_t i = 1; i <= 1000; ++i) {
        SensorFrame f{};
        f.timestamp_ns = i * 10000000ULL;
        const double t = static_cast<double>(i) * 0.01;
        const float vib = 1.2F * std::sin(2.0 * kPi * 25.0 * t);
        f.accel_m_s2 = {0.0F, 0.5F, static_cast<float>(kGravityStandard) + vib};
        f.gyro_rad_s = {0.01F, 0.0F, 0.0F};
        f.pressure_hpa = 1013.25F;
        f.has_imu = true;
        f.has_baro = true;

        const auto res = pipeline.processSensorFrame(f);
        if (res.has_value()) {
            last_state = *res;
        }
    }

    const auto pdr_pos = pipeline.getDeadReckoningPosition();
    std::cout << "[AUDIT CHECK] Activity: " << static_cast<int>(last_state.motion_activity)
              << " (" << last_state.activity_confidence << ") PDR Pos: (" << pdr_pos.easting_m << ", " << pdr_pos.northing_m 
              << ") m vs Particle Cluster 0: (" << last_state.clusters[0].center_easting_m << ", " 
              << last_state.clusters[0].center_northing_m << ") m" << std::endl;

    EXPECT_EQ(last_state.motion_activity, MotionActivity::kVehicleWheeled);
    EXPECT_GT(pdr_pos.northing_m, 110.0);
    EXPECT_GT(last_state.clusters[0].center_northing_m, 110.0);
    const double dist_pdr_pf = std::hypot(pdr_pos.easting_m - last_state.clusters[0].center_easting_m,
                                          pdr_pos.northing_m - last_state.clusters[0].center_northing_m);
    EXPECT_LT(dist_pdr_pf, 30.0);
}

// 2. Kiểm tra lỗi NaN Poisoning làm hỏng ESKF vĩnh viễn do thiếu IntegrityGuard
TEST(AuditDeepDiveTest, NanFramePermanentlyPoisonsEskfAttitude) {
    MultiModalFusionPipeline pipeline;
    environment::DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(1000.0, 1000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 400.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1500.0}, dem);

    // Bơm 1 frame nhiễm NaN/Inf vào pipeline
    constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
    constexpr float kInf = std::numeric_limits<float>::infinity();
    SensorFrame f_bad{};
    f_bad.timestamp_ns = 1000000000ULL;
    f_bad.accel_m_s2 = {kNan, 0.0F, 9.81F};
    f_bad.gyro_rad_s = {0.0F, kInf, 0.0F};
    f_bad.has_imu = true;
    pipeline.processSensorFrame(f_bad);

    // Sau đó bơm các frame hoàn toàn bình thường có bước chân
    bool nan_propagated_to_pdr = false;
    for (uint64_t i = 1; i <= 60; ++i) {
        SensorFrame f_good{};
        f_good.timestamp_ns = 1000000000ULL + i * 10000000ULL;
        float a_z = (i == 32) ? 14.81F : 9.81F;
        f_good.accel_m_s2 = {0.0F, 0.5F, a_z};
        f_good.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        f_good.pressure_hpa = 1013.25F;
        f_good.has_imu = true;
        f_good.has_baro = true;

        const auto res = pipeline.processSensorFrame(f_good);
        if (res.has_value()) {
            if (std::isnan(res->clusters[0].center_easting_m) ||
                std::isnan(pipeline.getDeadReckoningPosition().easting_m)) {
                nan_propagated_to_pdr = true;
            }
        }
    }

    std::cout << "[AUDIT CHECK] NaN Propagated to PDR / Cluster after bad frame: " 
              << (nan_propagated_to_pdr ? "YES (BUG DETECTED)" : "NO") << std::endl;

    EXPECT_FALSE(nan_propagated_to_pdr);
    EXPECT_FALSE(std::isnan(pipeline.getEskf().getAttitudeEstimate().euler.yaw_rad));
    EXPECT_FALSE(std::isnan(pipeline.getDeadReckoningPosition().easting_m));
}

// 3. Kiểm tra lỗi mất Barometer dẫn đến phân loại nhầm Đi bộ phẳng thành Leo cầu thang
TEST(AuditDeepDiveTest, FlatWalkingWithoutBarometerMisclassifiedAsStairs) {
    MotionActivityClassifier classifier{};
    Quaterniond q_level{1.0, 0.0, 0.0, 0.0};

    // Đi bộ phẳng 20 bước với has_baro = false (nhịp 1.6 Hz, rise time 220ms thông thường)
    size_t stair_ascent_count = 0;
    size_t walking_flat_count = 0;

    for (int step = 0; step < 20; ++step) {
        for (int i = 0; i < 60; ++i) {
            SensorFrame f{};
            f.timestamp_ns = static_cast<uint64_t>(step * 600000000ULL + i * 10000000ULL);
            float a_z = 9.81F;
            if (i >= 10 && i < 32) {
                a_z = 9.81F - 2.0F + (5.5F / 22.0F) * static_cast<float>(i - 10);
            } else if (i == 32) {
                a_z = 9.81F + 3.5F;
            }
            f.accel_m_s2 = {0.0F, 0.5F, a_z};
            f.gyro_rad_s = {0.05F, 0.0F, 0.0F};
            f.has_imu = true;
            f.has_baro = false; // KHÔNG CÓ KHÍ ÁP KẾ!

            const bool is_peak = (i == 32);
            const auto act = classifier.processSample(f, q_level, 1.6F, is_peak, 0.0, -1.0F, 0.01);
            if (is_peak) {
                if (act.current_activity == MotionActivity::kStairsAscent) {
                    stair_ascent_count++;
                } else if (act.current_activity == MotionActivity::kWalkingFlat) {
                    walking_flat_count++;
                }
            }
        }
    }

    std::cout << "[AUDIT CHECK] Without Barometer - Walking Steps classified as: StairsAscent = " 
              << stair_ascent_count << "/20, WalkingFlat = " << walking_flat_count << "/20" << std::endl;

    EXPECT_EQ(stair_ascent_count, 0U);
    EXPECT_GE(walking_flat_count, 18U);
}

// 4. Kiểm tra xe chạy tốc độ cao không kích hoạt cảnh báo sai kFaultVelocityExcess trong IntegrityMonitor
TEST(AuditDeepDiveTest, HighwayVehicleSpeedTriggersFalseVelocityFault) {
    IntegrityMonitor monitor{};
    CanonicalNavigationState state{};
    state.timestamp_ns = 1000000000ULL;
    // Xe chạy 80 km/h = 22.2 m/s
    state.velocity_m_s = {22.2, 0.0, 0.0};
    state.motion_activity.current_activity = MotionActivity::kVehicleWheeled;
    state.position_cov_2d = {10.0, 0.0, 0.0, 10.0};
    state.attitude_cov_3x3 = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

    const auto status = monitor.auditState(state, 1000000000ULL);
    std::cout << "[AUDIT CHECK] Vehicle at 80 km/h is_healthy: " << status.is_healthy 
              << ", fault_flags: 0x" << std::hex << status.fault_flags << std::dec << std::endl;

    EXPECT_TRUE(status.is_healthy);
    EXPECT_EQ(status.fault_flags, 0U);
    EXPECT_TRUE(status.velocity_valid);
}

// 5. Kiểm tra C API: Watchdog độ tươi cảm biến và cấu hình Lock-Free Triple Buffer
TEST(AuditDeepDiveTest, CApiPanicCirclingAndStaleFreshnessGaps) {
    survival::platform::initializePlatformHooks();
    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);

    // Đẩy 1 frame cảm biến
    SensorFrame f{};
    f.timestamp_ns = 1000000000ULL;
    f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    f.has_imu = true;
    survival_engine_push_sensor_frame(engine, &f);

    // 1. Kiểm tra trạng thái an toàn ban đầu qua Triple Buffer
    NavigationUncertaintyState state{};
    EXPECT_EQ(survival_engine_get_uncertainty_state(engine, &state), 1);
    EXPECT_FALSE(state.flags.panic_circling);

    // Kiểm tra tính tươi ban đầu: khỏe mạnh
    int is_healthy = 0;
    uint32_t fault_flags = 0xFFFFFFFF;
    survival_engine_get_integrity_status(engine, &is_healthy, &fault_flags);
    EXPECT_EQ(is_healthy, 1);
    EXPECT_EQ(fault_flags, 0U);

    // 2. Giả lập cảm biến dừng phát trong 10 giây (stale & dead)
    survival_engine_advance_simulated_time(engine, 10000000000ULL); // +10s
    survival_engine_get_integrity_status(engine, &is_healthy, &fault_flags);

    std::cout << "[AUDIT CHECK] CApi integrity status after 10s of silence: is_healthy=" 
              << is_healthy << ", faults=0x" << std::hex << fault_flags << std::dec << std::endl;

    EXPECT_EQ(is_healthy, 0);
    EXPECT_NE(fault_flags & survival::integrity::kFaultSensorDead, 0U);
    EXPECT_NE(fault_flags & survival::integrity::kFaultEstimatorStale, 0U);

    survival_engine_destroy(engine);
}
