#include <gtest/gtest.h>
#include "survival_core/environment/map_manager.hpp"
#include <cmath>

using namespace survival::environment;

TEST(MapManagerTest, MultipathPoisoningGatingAndRollback) {
    MapManager manager;

    // 1. Chuỗi mẫu GPS sạch (t = 0s đến 5s): std = 3m, HDOP = 1.0
    for (uint64_t t_sec = 0; t_sec <= 5; ++t_sec) {
        GpsSample clean_sample{};
        clean_sample.lat_rad = 0.3665;
        clean_sample.lon_rad = 1.8326;
        clean_sample.alt_m = 1200.0;
        clean_sample.std_pos_m = 3.5;
        clean_sample.hdop = 1.1;
        clean_sample.timestamp_ns = t_sec * 1000000000ULL;
        manager.handleGpsFix(clean_sample, 1200.0);
    }

    // 2. Nhiễm độc dội sóng Multipath trong hẻm núi (t = 6s đến 8s): std vọt lên 40m - 80m
    for (uint64_t t_sec = 6; t_sec <= 8; ++t_sec) {
        GpsSample dirty_sample{};
        dirty_sample.lat_rad = 0.3680; // Nhảy vọt toạ độ do dội sóng
        dirty_sample.lon_rad = 1.8340;
        dirty_sample.alt_m = 1350.0;
        dirty_sample.std_pos_m = 65.0; // std rất lớn
        dirty_sample.hdop = 4.5;
        dirty_sample.timestamp_ns = t_sec * 1000000000ULL;
        manager.handleGpsFix(dirty_sample, 1205.0);
    }

    // 3. Mất GPS tại t = 9s -> Kích hoạt handleGpsCutoff
    const auto handoff = manager.handleGpsCutoff(9000000000ULL);

    ASSERT_TRUE(handoff.is_valid);
    // Điểm neo sạch phải được lùi về mẫu t = 5s (std <= 12m), cao độ 1200m
    EXPECT_NEAR(handoff.anchor_pos.altitude_m, 1200.0, 1.0);
    EXPECT_LE(handoff.anchor_std_m, 12.0);

    // Thời gian cần chiếu tịnh tiến PDR phải bằng 9s - 5s = 4s = 4.000.000.000 ns
    EXPECT_EQ(handoff.time_to_project_pdr_ns, 4000000000ULL);
}

TEST(MapManagerTest, SoftBayesianGpsReacquisitionLikelihood) {
    // Giả sử GPS có lại tại (100m, 200m) với độ chính xác std = 5m
    const double gps_e = 100.0;
    const double gps_n = 200.0;
    const double gps_std = 5.0;

    // Hạt 1 nằm ngay tại tâm GPS -> Likelihood cực đại (ln w = 0)
    const double log_l_exact = MapManager::computeSoftGpsLogLikelihood(100.0, 200.0, gps_e, gps_n, gps_std);
    EXPECT_NEAR(log_l_exact, 0.0, 1e-6);

    // Hạt 2 nằm cách 10m (2*std) -> ln w = - (100) / (2 * 25) = -2.0
    const double log_l_2std = MapManager::computeSoftGpsLogLikelihood(110.0, 200.0, gps_e, gps_n, gps_std);
    EXPECT_NEAR(log_l_2std, -2.0, 1e-4);

    // Hạt 3 nằm cách 50m (10*std) -> ln w rất âm, bị phạt trọng số nặng
    const double log_l_far = MapManager::computeSoftGpsLogLikelihood(150.0, 200.0, gps_e, gps_n, gps_std);
    EXPECT_LT(log_l_far, -40.0);
}
