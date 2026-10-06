#include <gtest/gtest.h>
#include "survival_core/safety/resource_governor.hpp"

using namespace survival;
using namespace survival::safety;

TEST(ResourceGovernorTest, StateTransitionsAcrossBatteryLevels) {
    ResourceGovernor gov;

    // 1. Pin 85% -> Chế độ Balanced (N = 2000, 100 Hz)
    const auto s1 = gov.updateBatterySoc(85.0);
    EXPECT_EQ(s1.power_mode, PowerMode::kBalanced);
    EXPECT_EQ(s1.target_particle_count, 2000U);
    EXPECT_DOUBLE_EQ(s1.fast_loop_rate_hz, 100.0);
    EXPECT_TRUE(s1.allow_camera_sun);
    EXPECT_FALSE(s1.should_trigger_disk_flush);

    // 2. Pin 15% -> Chế độ Energy Conservative (N = 500, 50 Hz, tắt camera)
    const auto s2 = gov.updateBatterySoc(15.0);
    EXPECT_EQ(s2.power_mode, PowerMode::kEnergyConservative);
    EXPECT_EQ(s2.target_particle_count, 500U);
    EXPECT_DOUBLE_EQ(s2.fast_loop_rate_hz, 50.0);
    EXPECT_FALSE(s2.allow_camera_sun);
    EXPECT_FALSE(s2.should_trigger_disk_flush);

    // 3. Pin 4% -> Chế độ Critical Snapshot (N = 100, kích hoạt ghi Flash khẩn cấp)
    const auto s3 = gov.updateBatterySoc(4.0);
    EXPECT_EQ(s3.power_mode, PowerMode::kCriticalSnapshot);
    EXPECT_EQ(s3.target_particle_count, 100U);
    EXPECT_TRUE(s3.should_trigger_disk_flush);

    // 4. Pin 1% -> Chế độ Display Only (Tắt toàn bộ cảm biến để giữ màn hình)
    const auto s4 = gov.updateBatterySoc(1.0);
    EXPECT_EQ(s4.power_mode, PowerMode::kDisplayOnly);
    EXPECT_EQ(s4.target_particle_count, 0U);
    EXPECT_FALSE(s4.allow_sensors);
}


// [M10] Hysteresis 3%: pin dao dong quanh 20% khong duoc nhay mode
TEST(ResourceGovernorTest, HysteresisPreventsModeFlapping) {
    ResourceGovernor gov{};

    // Xuong 19% -> Conservative
    auto g1 = gov.updateBatterySoc(19.0);
    EXPECT_EQ(g1.power_mode, PowerMode::kEnergyConservative);

    // Len 21-22% van GIU Conservative (truoc fix: quay lai Balanced ngay)
    auto g2 = gov.updateBatterySoc(21.5);
    EXPECT_EQ(g2.power_mode, PowerMode::kEnergyConservative);
    auto g2b = gov.updateBatterySoc(22.9);
    EXPECT_EQ(g2b.power_mode, PowerMode::kEnergyConservative);

    // Chi tu 23% moi len Balanced
    auto g3 = gov.updateBatterySoc(23.0);
    EXPECT_EQ(g3.power_mode, PowerMode::kBalanced);

    // Tuong tu quanh 5%: xuong Critical tai <=5, len lai phai qua 8%
    auto g4 = gov.updateBatterySoc(4.0);   // Conservative -> Critical
    EXPECT_EQ(g4.power_mode, PowerMode::kCriticalSnapshot);
    auto g5 = gov.updateBatterySoc(6.0);
    EXPECT_EQ(g5.power_mode, PowerMode::kCriticalSnapshot);  // truoc fix: Conservative
    auto g6 = gov.updateBatterySoc(8.0);
    EXPECT_EQ(g6.power_mode, PowerMode::kEnergyConservative);
}
