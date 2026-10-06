#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/dem_map.hpp"
#include <vector>

using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;

TEST(DeterministicReplayTest, IdenticalSeedYieldsExactSameState) {
    constexpr uint32_t kSeed = 0xDEADBEEF;

    PipelineConfig cfg1{};
    cfg1.random_seed = kSeed;
    MultiModalFusionPipeline pipeline1(cfg1);
    pipeline1.initialize(EastingNorthingAlt{100.0, 200.0, 1500.0}, DemMap{});

    PipelineConfig cfg2{};
    cfg2.random_seed = kSeed;
    MultiModalFusionPipeline pipeline2(cfg2);
    pipeline2.initialize(EastingNorthingAlt{100.0, 200.0, 1500.0}, DemMap{});

    // Tạo 100 frames giả lập
    std::vector<SensorFrame> frames(100);
    for (size_t i = 0; i < 100; ++i) {
        frames[i].timestamp_ns = 1000000000ULL + i * 10000000ULL;
        frames[i].accel_m_s2 = {0.0F, 0.0F, 9.80665F + static_cast<float>(std::sin(static_cast<double>(i) * 0.1) * 2.0)};
        frames[i].gyro_rad_s = {0.0F, 0.0F, 0.02F};
        frames[i].mag_uT = {0.0F, 30.0F, -35.0F};
        frames[i].pressure_hpa = 850.0F;
        frames[i].has_imu = true;
        frames[i].has_mag = true;
        frames[i].has_baro = true;
    }

    std::optional<NavigationUncertaintyState> last_state1 = std::nullopt;
    std::optional<NavigationUncertaintyState> last_state2 = std::nullopt;

    for (const auto& f : frames) {
        auto res1 = pipeline1.processSensorFrame(f);
        if (res1) last_state1 = res1;

        auto res2 = pipeline2.processSensorFrame(f);
        if (res2) last_state2 = res2;
    }

    ASSERT_TRUE(last_state1.has_value());
    ASSERT_TRUE(last_state2.has_value());

    // Khẳng định 100% Deterministic: Hai lần chạy cho kết quả chính xác đến từng bit
    EXPECT_DOUBLE_EQ(last_state1->clusters[0].center_easting_m, last_state2->clusters[0].center_easting_m);
    EXPECT_DOUBLE_EQ(last_state1->clusters[0].center_northing_m, last_state2->clusters[0].center_northing_m);
    EXPECT_DOUBLE_EQ(last_state1->clusters[0].center_altitude_m, last_state2->clusters[0].center_altitude_m);
    EXPECT_DOUBLE_EQ(last_state1->clusters[0].ce95_semi_major_m, last_state2->clusters[0].ce95_semi_major_m);
    EXPECT_DOUBLE_EQ(last_state1->global_navigation_score, last_state2->global_navigation_score);
}
