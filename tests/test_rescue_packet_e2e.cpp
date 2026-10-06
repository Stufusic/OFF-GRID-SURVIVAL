#include <gtest/gtest.h>
#include "survival_core/comms/packet_serializer.hpp"
#include "survival_core/comms/sos_egress_manager.hpp"
#include "survival_core/environment/map_manager.hpp"
#include "survival_core/types.hpp"
#include <cmath>
#include <string>

using namespace survival;
using namespace survival::comms;
using namespace survival::environment;

TEST(RescuePacketE2ETest, FullRoundTripWithRealCoordinates) {
    // 1. Tọa độ thực tế vùng núi Đà Lạt / Langbiang: 11.940419° N, 108.458313° E, Cao độ 1500m
    constexpr double true_lat_deg = 11.940419;
    constexpr double true_lon_deg = 108.458313;
    constexpr double true_alt_m = 1500.0;

    LocalFrame frame{
        .origin_lat_rad = true_lat_deg * kDegToRad,
        .origin_lon_rad = true_lon_deg * kDegToRad,
        .origin_alt_m = true_alt_m
    };

    MapManager manager(frame);

    // Bơm GPS Fix chuẩn
    GpsSample gps{
        .lat_rad = true_lat_deg * kDegToRad,
        .lon_rad = true_lon_deg * kDegToRad,
        .alt_m = true_alt_m,
        .std_pos_m = 2.5,
        .hdop = 0.9,
        .timestamp_ns = 1700000000000000000ULL,
        .is_valid = true
    };
    manager.handleGpsFix(gps, 1500.0);

    const auto& last_fix = manager.getLastKnownGoodFix();
    ASSERT_TRUE(last_fix.is_valid);
    EXPECT_NEAR(last_fix.latitude_deg, true_lat_deg, 1e-5);
    EXPECT_NEAR(last_fix.longitude_deg, true_lon_deg, 1e-5);

    // 2. Mã hóa sang gói tin cứu hộ 160-bit (20 bytes)
    const auto packet_bytes = Packet160Serializer::encode(
        static_cast<uint32_t>(last_fix.timestamp_ns / 1000000000ULL),
        last_fix.latitude_deg,
        last_fix.longitude_deg,
        last_fix.altitude_m,
        25.0, // CE50 Major
        15.0, // CE50 Minor
        45.0, // Heading
        static_cast<uint8_t>(NavigationMode::kGpsAided),
        85,   // Battery 85%
        1     // Cluster ID 1
    );

    // 3. Giải mã và kiểm tra CRC32
    DecodedRescuePacket decoded{};
    const bool crc_ok = Packet160Serializer::decode(packet_bytes, decoded);
    EXPECT_TRUE(crc_ok);
    EXPECT_TRUE(decoded.is_crc_valid);

    // 4. [SAFETY INVARIANT 2]: Khẳng định tọa độ không được là Null Island (0.0, 0.0)
    EXPECT_NE(decoded.latitude_deg, 0.0);
    EXPECT_NE(decoded.longitude_deg, 0.0);

    // Kiểm tra độ chính xác sau lượng tử hóa 24-bit fixed point
    EXPECT_NEAR(decoded.latitude_deg, true_lat_deg, 0.0001);   // Sai số lượng tử < 1m
    EXPECT_NEAR(decoded.longitude_deg, true_lon_deg, 0.0001);
    EXPECT_NEAR(decoded.altitude_m, true_alt_m, 1.0);
    EXPECT_EQ(decoded.cluster_id, 1U);

    // 5. Kiểm tra định dạng SMS Egress
    NavigationUncertaintyState state{};
    state.clusters[0].center_altitude_m = true_alt_m;
    state.clusters[0].ce95_semi_major_m = 48.0;
    state.nav_mode = NavigationMode::kGpsAided;
    state.position_age_s = 2.0;

    OutboundRescuePacket160 out_pkt{};
    out_pkt.data = packet_bytes;

    const std::string sms = SosEgressManager::buildCompactSmsPayload(out_pkt, state, 85.0);
    EXPECT_NE(sms.find("LAT:11.9404"), std::string::npos);
    EXPECT_NE(sms.find("LON:108.4583"), std::string::npos);
    EXPECT_NE(sms.find("CE95:48m"), std::string::npos);
    EXPECT_NE(sms.find("https://maps.google.com/"), std::string::npos);
}
