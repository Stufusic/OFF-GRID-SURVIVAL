#include <gtest/gtest.h>
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

using namespace survival;
using namespace survival::environment;
using namespace survival::math;

TEST(DemMapTest, SyntheticConeElevationAndGradient) {
    DemMap dem;
    // Tạo đỉnh núi nón: Bán kính 500m, Đỉnh 2000m tại (1000, 1000), Chân núi 1000m
    const bool ok = dem.createSyntheticCone(2000.0, 2000.0, 10.0, 1000.0, 1000.0, 2000.0, 1000.0, 500.0);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(dem.isLoaded());

    // 1. Kiểm tra cao độ đỉnh núi (1000, 1000)
    const auto peak_elev = dem.getElevationBilinear(1000.0, 1000.0);
    ASSERT_TRUE(peak_elev.has_value());
    EXPECT_NEAR(*peak_elev, 2000.0, 1.0);

    // 2. Kiểm tra cao độ chân núi ngoài bán kính 500m (ví dụ tại 200, 200)
    const auto base_elev = dem.getElevationBilinear(200.0, 200.0);
    ASSERT_TRUE(base_elev.has_value());
    EXPECT_NEAR(*base_elev, 1000.0, 1.0);

    // 3. Kiểm tra cao độ sườn núi lưng chừng tại d = 250m (ví dụ tại 1000, 1250)
    const auto mid_elev = dem.getElevationBilinear(1000.0, 1250.0);
    ASSERT_TRUE(mid_elev.has_value());
    EXPECT_NEAR(*mid_elev, 1500.0, 5.0);

    // 4. Kiểm tra Gradient và Hướng sườn dốc (Aspect) tại phía Bắc đỉnh núi (1000, 1250)
    // Sườn dốc xuống về phía Bắc -> Aspect phải xấp xỉ 0 độ (Bắc)
    const auto grad_north = dem.getGradient(1000.0, 1250.0);
    ASSERT_TRUE(grad_north.is_valid);
    EXPECT_GT(grad_north.slope_rad, 0.5); // tan(slope) = 1000/500 = 2.0 -> slope ~ 63 độ
    EXPECT_NEAR(radToDeg(grad_north.aspect_rad), 0.0, 5.0);
}

TEST(DemMapTest, FlatTerrainAspectDegeneracy) {
    DemMap dem;
    // Tạo bề mặt phẳng hoàn toàn
    std::vector<float> flat_data(100 * 100, 1200.0F);
    DemHeader header{
        .origin_easting_m = 0.0,
        .origin_northing_m = 0.0,
        .origin_lat_rad = 0.0,
        .origin_lon_rad = 0.0,
        .cell_size_m = 10.0,
        .width_cells = 100,
        .height_cells = 100
    };
    ASSERT_TRUE(dem.loadFromMemory(header, flat_data));

    const auto grad = dem.getGradient(500.0, 500.0);
    ASSERT_TRUE(grad.is_valid);
    EXPECT_NEAR(grad.gradient_norm, 0.0, 1e-6);
    EXPECT_NEAR(grad.slope_rad, 0.0, 1e-6);
    EXPECT_TRUE(dem.isTraversable(500.0, 500.0));
}

TEST(DemMapTest, TraversabilityCliffDetection) {
    DemMap dem;
    // Dốc đứng 1000m trên 200m cự ly -> tan(slope) = 5.0 -> slope = 78 độ (> 55 độ)
    ASSERT_TRUE(dem.createSyntheticRamp(500.0, 500.0, 10.0, 1000.0, 3500.0));

    const auto grad = dem.getGradient(250.0, 250.0);
    ASSERT_TRUE(grad.is_valid);
    EXPECT_GT(grad.slope_rad, degToRad(55.0)); // > 55 độ
    EXPECT_FALSE(dem.isTraversable(250.0, 250.0, degToRad(55.0))); // Vách đá không thể đi qua
}

TEST(DemMapTest, LocalEnuWgs84RoundTrip) {
    DemMap dem;
    DemHeader header{
        .origin_easting_m = 0.0,
        .origin_northing_m = 0.0,
        .origin_lat_rad = degToRad(21.0285), // Hà Nội ~ 21 độ Bắc
        .origin_lon_rad = degToRad(105.8542),
        .cell_size_m = 10.0,
        .width_cells = 100,
        .height_cells = 100
    };
    std::vector<float> data(10000, 500.0F);
    ASSERT_TRUE(dem.loadFromMemory(header, data));

    const double test_e = 500.0;
    const double test_n = 750.0;

    const auto [lat, lon] = dem.enuToLatLon(test_e, test_n);
    const auto [re_e, re_n] = dem.latLonToEnu(lat, lon);

    EXPECT_NEAR(re_e, test_e, 1e-4);
    EXPECT_NEAR(re_n, test_n, 1e-4);
}
