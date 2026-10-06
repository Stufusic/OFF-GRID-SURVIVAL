#include <gtest/gtest.h>
#include "survival_core/integrity/integrity_monitor.hpp"
#include "survival_core/safety/resource_governor.hpp"
#include "survival_core/environment/map_manager.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include <cmath>

using namespace survival;
using namespace survival::integrity;
using namespace survival::safety;
using namespace survival::environment;
using namespace survival::comms;

TEST(SafetyInvariantsTest, Invariant7_NoFalseConfidence) {
    IntegrityMonitor monitor{};

    CanonicalNavigationState state{};
    state.timestamp_ns = 1000000000ULL;
    state.global_navigation_score = 0.95; // Báo 95% độ tin cậy

    // Nhưng tất cả cảm biến đều bị suy giảm (IMU = 0.1, Mag = 0.1, Baro = 0.1)
    state.sensor_health.imu = 0.1F;
    state.sensor_health.magnetometer = 0.1F;
    state.sensor_health.barometer = 0.1F;

    const auto status = monitor.auditState(state, 1000000000ULL);
    EXPECT_FALSE(status.is_healthy);
    EXPECT_TRUE((status.fault_flags & kFaultFalseConfidence) != 0);
}

TEST(SafetyInvariantsTest, Invariant8_CovarianceValidityCheck) {
    IntegrityMonitor monitor{};

    // 1. Ma trận đối xứng và xác định dương
    EXPECT_TRUE(IntegrityMonitor::checkCovariance2D(10.0, 10.0, 2.0));

    // 2. Ma trận không xác định dương (det < 0: var_e * var_n < cov_en^2)
    EXPECT_FALSE(IntegrityMonitor::checkCovariance2D(5.0, 5.0, 10.0));

    // 3. Ma trận chứa NaN / Inf
    EXPECT_FALSE(IntegrityMonitor::checkCovariance2D(std::numeric_limits<double>::quiet_NaN(), 10.0, 0.0));
}

TEST(SafetyInvariantsTest, Invariant10_NoNanInfInCanonicalState) {
    IntegrityMonitor monitor{};

    CanonicalNavigationState state{};
    state.position_enu.easting_m = std::numeric_limits<double>::quiet_NaN();

    const auto status = monitor.auditState(state, 1000000000ULL);
    EXPECT_FALSE(status.is_healthy);
    EXPECT_TRUE((status.fault_flags & kFaultNanInfDetected) != 0);
}

TEST(SafetyInvariantsTest, Invariant11_EstimatorFreshnessWatchdog) {
    IntegrityMonitor monitor{};

    CanonicalNavigationState state{};
    state.timestamp_ns = 1000000000ULL; // t = 1.0s

    // Thời gian hiện tại t = 3.0s (quá 1.5s timeout)
    const auto status = monitor.auditState(state, 3000000000ULL);
    EXPECT_FALSE(status.is_healthy);
    EXPECT_TRUE((status.fault_flags & kFaultEstimatorStale) != 0);
}

TEST(SafetyInvariantsTest, Invariant12_RescuePacketProvenanceAndAge) {
    const auto bytes = Packet160Serializer::encode(
        1700000000U,
        11.940419,
        108.458313,
        1500.0,
        25.0,
        15.0,
        90.0,
        static_cast<uint8_t>(NavigationMode::kTrnAided),
        85,
        2
    );

    DecodedRescuePacket decoded{};
    const bool ok = Packet160Serializer::decode(bytes, decoded);
    EXPECT_TRUE(ok);
    EXPECT_EQ(decoded.status_flags, static_cast<uint8_t>(NavigationMode::kTrnAided));
    EXPECT_EQ(decoded.cluster_id, 2U);
    EXPECT_NEAR(decoded.battery_percent, 85U, 2U);
    EXPECT_TRUE(decoded.is_crc_valid);
}
