#include <gtest/gtest.h>
#include "survival_core/estimators/particle_filter_engine.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;
using namespace survival::math;

TEST(ParticleFilterTest, InitializationAndMotionPropagation) {
    ParticleFilterConfig config;
    config.num_particles = 1000;
    config.init_pos_std_m = 5.0;

    ParticleFilterEngine pf(config, 42);
    pf.initialize(EastingNorthingAlt{500.0, 500.0, 1500.0}, 5.0);

    // 1. Kiểm tra tâm phân bố ban đầu
    const auto init_mean = pf.getWeightedMeanPosition();
    EXPECT_NEAR(init_mean.easting_m, 500.0, 1.0);
    EXPECT_NEAR(init_mean.northing_m, 500.0, 1.0);

    // 2. Lan truyền 10 bước đi về hướng Bắc (L = 0.75m, psi = 0)
    StepEvent step{};
    step.stride_mean_m = 0.75;
    step.stride_variance_m2 = 0.0016; // sigma = 0.04m
    step.heading_rad = 0.0;           // Hướng Bắc
    step.heading_variance_rad2 = 0.01;
    step.vertical_displacement_m = 0.0;

    for (int i = 0; i < 10; ++i) {
        pf.predict(step);
    }

    const auto prop_mean = pf.getWeightedMeanPosition();
    // Sau 10 bước: N = 500 + 7.5m = 507.5m, E ~ 500m
    EXPECT_NEAR(prop_mean.northing_m, 507.5, 1.0);
    EXPECT_NEAR(prop_mean.easting_m, 500.0, 2.0);
}

TEST(ParticleFilterTest, TerrainElevationWeightingAndResampling) {
    // 1. Tạo bản đồ đỉnh núi nón tại (1000, 1000), Đỉnh 2000m, Chân núi 1000m
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(2000.0, 2000.0, 10.0, 1000.0, 1000.0, 2000.0, 1000.0, 500.0));

    ParticleFilterConfig config;
    config.num_particles = 2000;
    config.init_pos_std_m = 200.0; // Phân tán rộng

    ParticleFilterEngine pf(config, 101);
    pf.initialize(EastingNorthingAlt{1000.0, 1000.0, 1500.0}, 200.0);

    // 2. Cập nhật đo lường với áp kế chỉ đúng độ cao đỉnh núi (2000m)
    // Các hạt ở gần đỉnh (2000m) sẽ nhận trọng số cực lớn
    for (int iter = 0; iter < 5; ++iter) {
        pf.updateWeights(dem, 2000.0, 0.0, 0.0);
    }

    // 3. Tâm của đám mây hạt phải hội tụ rất sát đỉnh núi (1000, 1000)
    const auto conv_mean = pf.getWeightedMeanPosition();
    EXPECT_NEAR(conv_mean.easting_m, 1000.0, 40.0);
    EXPECT_NEAR(conv_mean.northing_m, 1000.0, 40.0);

    // Trọng số sau tái lấy mẫu phải được chuẩn hóa đều đặn
    for (const auto& p : pf.getParticles()) {
        EXPECT_NEAR(p.weight, 1.0 / 2000.0, 1e-6);
        EXPECT_TRUE(p.is_alive);
    }
}

TEST(ParticleFilterTest, OutOfBoundsCullingAndRecovery) {
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(200.0, 200.0, 10.0, 1000.0, 1500.0));

    ParticleFilterConfig config;
    config.num_particles = 500;
    config.neff_resample_ratio = 0.90; // Kích hoạt tái lấy mẫu sớm

    ParticleFilterEngine pf(config, 77);
    // Khởi tạo sát biên Đông (E = 190m)
    pf.initialize(EastingNorthingAlt{190.0, 100.0, 1200.0}, 20.0);

    // Đẩy các hạt về phía Đông ra khỏi bản đồ
    StepEvent step_east{};
    step_east.stride_mean_m = 25.0; // Bước lớn vượt biên
    step_east.stride_variance_m2 = 1.0;
    step_east.heading_rad = kHalfPi; // Hướng Đông
    step_east.heading_variance_rad2 = 0.01;

    pf.predict(step_east);
    pf.updateWeights(dem, 1250.0, 0.0, kHalfPi);

    // Sau khi tái lấy mẫu, toàn bộ hạt sống sót phải nằm bên trong bản đồ
    for (const auto& p : pf.getParticles()) {
        EXPECT_TRUE(dem.isInside(p.easting_m, p.northing_m));
        EXPECT_TRUE(p.is_alive);
    }
}

// ============================================================================
// [C1] setTargetParticleCount phải bảo toàn hình thái phân phối
// ============================================================================
#include <unordered_map>
#include <cstdint>

namespace {
// Đa số nhiều nhất các hạt trùng tuyệt đối (lượng tử hóa 1 cm) trong đám mây
int maxDuplicateMultiplicity(const std::vector<Particle>& parts) {
    std::unordered_map<uint64_t, int> grid;
    int max_mult = 0;
    for (const auto& p : parts) {
        const auto e = static_cast<uint32_t>(std::llround(p.easting_m * 100.0));
        const auto n = static_cast<uint32_t>(std::llround(p.northing_m * 100.0));
        const uint64_t key = (static_cast<uint64_t>(e) << 32) | static_cast<uint64_t>(n);
        const int mult = ++grid[key];
        if (mult > max_mult) { max_mult = mult; }
    }
    return max_mult;
}
} // namespace

TEST(ParticleFilterResize, GrowPreservesDistributionSpread) {
    ParticleFilterConfig config;
    config.num_particles = 2000;
    config.init_pos_std_m = 20.0;
    ParticleFilterEngine pf(config, 42);
    pf.initialize(EastingNorthingAlt{500.0, 500.0, 1200.0}, 20.0);

    const auto cov_before = pf.getWeightedCovariance2D();
    const double std_before = std::sqrt(cov_before[0]);
    const auto mean_before = pf.getWeightedMeanPosition();

    pf.setTargetParticleCount(10000);

    EXPECT_EQ(pf.getParticleCount(), 10000U);
    EXPECT_NEAR(pf.getEffectiveParticleCount(), 10000.0, 1e-9);

    // Trước fix [C1]: 8000 hạt = bản sao hạt[0] -> std sụp ~5x, N_eff = N
    // khiến roughening/resample không bao giờ chạy lại.
    const auto cov_after = pf.getWeightedCovariance2D();
    EXPECT_GE(std::sqrt(cov_after[0]), 0.5 * std_before);

    // Tâm phân phối không được dịch chuyển quá 3 m
    const auto mean_after = pf.getWeightedMeanPosition();
    EXPECT_NEAR(mean_after.easting_m, mean_before.easting_m, 3.0);
    EXPECT_NEAR(mean_after.northing_m, mean_before.northing_m, 3.0);

    // Không được tồn tại cụm bản sao trùng tuyệt đối (old code: 8000 hạt trùng)
    EXPECT_LT(maxDuplicateMultiplicity(pf.getParticles()), 20);
}

TEST(ParticleFilterResize, ShrinkKeepsDistributionCenter) {
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(2000.0, 2000.0, 10.0, 1000.0, 1000.0, 2000.0, 1000.0, 500.0));

    ParticleFilterConfig config;
    config.num_particles = 2000;
    config.init_pos_std_m = 300.0;
    ParticleFilterEngine pf(config, 7);
    pf.initialize(EastingNorthingAlt{1000.0, 1000.0, 1500.0}, 300.0);
    for (int iter = 0; iter < 4; ++iter) {
        pf.updateWeights(dem, 2000.0, 0.0, 0.0);
    }

    const auto mean_before = pf.getWeightedMeanPosition();
    pf.setTargetParticleCount(500);
    const auto mean_after = pf.getWeightedMeanPosition();

    EXPECT_EQ(pf.getParticleCount(), 500U);
    // Shrink phải chọn đại diện có trọng số: tâm không trôi xa (trước fix: cắt
    // 500 phần tử ĐẦU mảng — sai lệch ngẫu nhiên nhưng có thể lớn)
    EXPECT_NEAR(mean_after.easting_m, mean_before.easting_m, 40.0);
    EXPECT_NEAR(mean_after.northing_m, mean_before.northing_m, 40.0);

    // Trọng số sau resize phải chuẩn hóa đều
    for (const auto& p : pf.getParticles()) {
        EXPECT_NEAR(p.weight, 1.0 / 500.0, 1e-9);
    }
}


// ============================================================================
// [B1] Ellipse cum theo hiep phuong sai that + merge o ke
// ============================================================================
TEST(PfClusterCovariance, EllipseMatchesTrueSpread) {
    // Init std 5 m -> CE95 = 2.4477 * 5 = 12.2 m (truoc fix: 122.4m co dinh!)
    ParticleFilterConfig config;
    config.num_particles = 2000;
    config.init_pos_std_m = 5.0;
    ParticleFilterEngine pf(config, 99);
    pf.initialize(EastingNorthingAlt{1000.0, 1000.0, 1200.0}, 5.0);

    const auto st = pf.extractUncertaintyState(1000000000ULL);
    ASSERT_GE(st.active_clusters_count, 1U);
    EXPECT_NEAR(st.clusters[0].ce95_semi_major_m, 12.2, 5.0);  // ±40%
    EXPECT_LT(st.clusters[0].ce95_semi_major_m, 40.0);          // cam tran 122m cu
    EXPECT_GT(st.clusters[0].ce95_semi_minor_m, 0.0);
}

TEST(PfClusterCovariance, StraddleBoundaryMergesToOneCluster) {
    // Dam may std 1.5m tam dung tren BIEN o (x = 1000 voi cell 10m) — truoc fix
    // bi tach thanh 2 cum gia thuyet ~50/50
    ParticleFilterConfig config;
    config.num_particles = 2000;
    config.init_pos_std_m = 1.5;
    config.grid_cluster_cell_size_m = 10.0;
    ParticleFilterEngine pf(config, 7);
    pf.initialize(EastingNorthingAlt{1000.0, 1000.0, 1200.0}, 1.5);

    const auto st = pf.extractUncertaintyState(1000000000ULL);
    EXPECT_EQ(st.active_clusters_count, 1U);
    EXPECT_NEAR(st.clusters[0].center_easting_m, 1000.0, 1.0);
    EXPECT_NEAR(st.clusters[0].center_northing_m, 1000.0, 1.0);
    // Sau merge: phan tan day du duoc bao toan
    EXPECT_NEAR(st.clusters[0].ce95_semi_major_m, 2.4477 * 1.5, 1.5);
}
