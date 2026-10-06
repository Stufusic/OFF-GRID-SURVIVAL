#include <gtest/gtest.h>
#include "survival_core/data_contracts.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include "survival_core/integrity/integrity_guard.hpp"
#include <cmath>

using namespace survival;
using namespace survival::comms;
using namespace survival::integrity;

TEST(DataContractsTest, OutboundRescuePacketSize) {
    EXPECT_EQ(sizeof(OutboundRescuePacket160), 20U);
    EXPECT_EQ(Packet160Serializer::kPacketSizeBytes, 20U);
}

TEST(Packet160SerializerTest, EncodeDecodeRoundTrip) {
    const uint32_t timestamp = 1725184800; // Mốc thời gian thực UTC
    const double lat = 11.940419;          // Tọa độ Vĩ độ Đà Lạt
    const double lon = 108.458313;         // Tọa độ Kinh độ Đà Lạt
    const double alt = 1500.0;             // Cao độ [m]
    const double ce50_major = 45.0;        // CE50 bán trục lớn [m]
    const double ce50_minor = 30.0;        // CE50 bán trục nhỏ [m]
    const double heading = 135.0;          // Hướng Đông Nam [deg]
    const uint8_t flags = 0x85;            // SOS=1, IMU=1, BARO=1
    const uint8_t battery = 75;            // 75%
    const uint8_t cluster_id = 2;

    // 1. Mã hóa sang 20 bytes
    const auto packet_bytes = Packet160Serializer::encode(
        timestamp, lat, lon, alt, ce50_major, ce50_minor, heading, flags, battery, cluster_id
    );

    // 2. Giải mã ngược lại
    DecodedRescuePacket decoded{};
    const bool ok = Packet160Serializer::decode(packet_bytes, decoded);

    EXPECT_TRUE(ok);
    EXPECT_TRUE(decoded.is_crc_valid);
    EXPECT_EQ(decoded.unix_timestamp, timestamp);
    EXPECT_NEAR(decoded.latitude_deg, lat, 0.0001);   // Độ phân giải 24-bit ~ 1 mét
    EXPECT_NEAR(decoded.longitude_deg, lon, 0.0001);
    EXPECT_NEAR(decoded.altitude_m, alt, 1.0);
    EXPECT_NEAR(decoded.ce50_major_m, ce50_major, 1.0);
    EXPECT_NEAR(decoded.ce50_minor_ratio, 30.0 / 45.0, 0.1);
    EXPECT_NEAR(decoded.ce_heading_deg, heading, 2.0); // Độ phân giải 8-bit ~ 1.4 độ
    EXPECT_EQ(decoded.status_flags, flags);
    EXPECT_NEAR(decoded.battery_percent, battery, 4.0); // Độ phân giải 5-bit ~ 3.2%
    EXPECT_EQ(decoded.cluster_id, cluster_id);
}

TEST(Packet160SerializerTest, CrcCorruptionRejection) {
    auto packet_bytes = Packet160Serializer::encode(
        1000, 10.0, 100.0, 500.0, 20.0, 10.0, 45.0, 0, 100, 0
    );

    // Cố tình làm sai lệch 1 byte trong payload
    packet_bytes[5] ^= 0xFF;

    DecodedRescuePacket decoded{};
    const bool ok = Packet160Serializer::decode(packet_bytes, decoded);

    EXPECT_FALSE(ok);
    EXPECT_FALSE(decoded.is_crc_valid);
}

TEST(IntegrityGuardTest, SanitizeNanAndInfinity) {
    IntegrityGuard guard;
    SensorFrame frame{};
    frame.accel_m_s2[0] = std::numeric_limits<float>::quiet_NaN();
    frame.accel_m_s2[1] = 0.0F;
    frame.accel_m_s2[2] = std::numeric_limits<float>::infinity();
    frame.pressure_hpa = 200.0F; // Dưới giới hạn vật lý 300 hPa

    const bool valid = guard.sanitizeSensorFrame(frame);
    EXPECT_FALSE(valid);
    EXPECT_FALSE(frame.has_imu);
    EXPECT_FALSE(frame.has_baro);
    EXPECT_FALSE(std::isnan(frame.accel_m_s2[0]));
    EXPECT_FALSE(std::isinf(frame.accel_m_s2[2]));
    EXPECT_EQ(guard.getTelemetry().nan_inf_detected, 2U);
    EXPECT_EQ(guard.getTelemetry().baro_out_of_bounds, 1U);
}

TEST(IntegrityGuardTest, EnforcePsd3x3NonPositiveDefinite) {
    IntegrityGuard guard;
    // Ma trận không xác định dương (có trị riêng âm):
    // [ 1.0   2.0   0.0 ]
    // [ 2.0   1.0   0.0 ]  -> Trị riêng: 3.0, -1.0, 1.0 (có trị riêng âm -1.0)
    // [ 0.0   0.0   1.0 ]
    std::array<double, 9> P_bad = {
        1.0, 2.0, 0.0,
        2.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    const bool repaired = guard.enforcePsd3x3(P_bad, 1e-4);
    EXPECT_TRUE(repaired);
    EXPECT_GT(guard.getTelemetry().psd_covariance_repairs, 0U);

    // Sau khi sửa, ma trận phải đối xứng và các phần tử trên đường chéo phải dương
    EXPECT_NEAR(P_bad[1], P_bad[3], 1e-9); // Đối xứng
    EXPECT_GT(P_bad[0], 0.0);
    EXPECT_GT(P_bad[4], 0.0);
    EXPECT_GT(P_bad[8], 0.0);
}
