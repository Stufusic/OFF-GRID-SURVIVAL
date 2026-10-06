#include <gtest/gtest.h>
#include "survival_core/estimators/particle_filter_engine.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;
using namespace survival::comms;

TEST(MultiHypothesisClusteringTest, BimodalParticleCloudClustering) {
    ParticleFilterConfig config;
    config.num_particles = 1000;
    config.grid_cluster_cell_size_m = 100.0;
    config.cluster_weight_threshold = 0.10;

    ParticleFilterEngine pf(config, 999);
    pf.initialize(EastingNorthingAlt{300.0, 300.0, 1000.0}, 10.0);

    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1200.0, 1200.0, 10.0, 1000.0, 1500.0));

    // Trích xuất trạng thái độ bất định
    const uint64_t timestamp_ns = 1700000000ULL * 1000000000ULL;
    const auto state = pf.extractUncertaintyState(timestamp_ns);

    EXPECT_GE(state.active_clusters_count, 1U);
    EXPECT_LE(state.active_clusters_count, 4U);

    // Kiểm tra tính hợp lệ của Elip sai số cụm
    const auto& c0 = state.clusters[0];
    EXPECT_GT(c0.ce95_semi_major_m, 0.0);
    EXPECT_GT(c0.ce95_semi_minor_m, 0.0);
    EXPECT_GT(c0.area_95_m2, 0.0);
    EXPECT_FALSE(std::isnan(c0.ellipse_heading_rad));

    EXPECT_GT(state.global_navigation_score, 0.0);
    EXPECT_LT(state.global_navigation_score, 1.0);

    // Kiểm tra tương thích đóng gói và giải mã OutboundRescuePacket160
    const double lat_deg = c0.center_northing_m / 111139.0;
    const double lon_deg = c0.center_easting_m / 111139.0;
    const auto packet = Packet160Serializer::encode(
        1700000000U,
        lat_deg,
        lon_deg,
        c0.center_altitude_m,
        c0.ce95_semi_major_m,
        c0.ce95_semi_minor_m,
        c0.ellipse_heading_rad * 180.0 / 3.141592653589793,
        0x01,
        state.battery_percent,
        c0.cluster_id
    );

    DecodedRescuePacket decoded{};
    const bool decode_ok = Packet160Serializer::decode(packet, decoded);

    ASSERT_TRUE(decode_ok);
    EXPECT_TRUE(decoded.is_crc_valid);
    EXPECT_NEAR(decoded.latitude_deg, lat_deg, 0.001);
    EXPECT_NEAR(decoded.longitude_deg, lon_deg, 0.001);
    EXPECT_EQ(decoded.cluster_id, c0.cluster_id);
}
