#include <gtest/gtest.h>
#include "survival_core/environment/dem_map.hpp"
#include "survival_platform/platform_factory.hpp"
#include <cstdio>

using namespace survival::environment;

TEST(DemMmapIoTest, SdemSaveAndLoadMmapRoundTrip) {
    survival::platform::initializePlatformHooks();

    DemMap dem_orig;
    ASSERT_TRUE(dem_orig.createSyntheticCone(1000.0, 1000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 400.0));

    const std::string temp_file = "test_synthetic_cone.sdem";
    ASSERT_TRUE(dem_orig.saveToFile(temp_file));

    // Nạp qua Memory Mapping
    DemMap dem_loaded;
    ASSERT_TRUE(dem_loaded.loadFromFile(temp_file, true));
    EXPECT_TRUE(dem_loaded.isLoaded());
    EXPECT_TRUE(dem_loaded.isMemoryMapped());

    EXPECT_EQ(dem_loaded.getHeader().width_cells, dem_orig.getHeader().width_cells);
    EXPECT_EQ(dem_loaded.getHeader().height_cells, dem_orig.getHeader().height_cells);

    // Kiểm tra cao độ tại tâm đỉnh núi (500m, 500m)
    const auto alt_orig = dem_orig.getElevationBilinear(500.0, 500.0);
    const auto alt_loaded = dem_loaded.getElevationBilinear(500.0, 500.0);

    ASSERT_TRUE(alt_orig.has_value());
    ASSERT_TRUE(alt_loaded.has_value());
    EXPECT_NEAR(*alt_loaded, *alt_orig, 1e-4);
    EXPECT_NEAR(*alt_loaded, 2000.0, 1.0);

    // Dọn dẹp tệp tạm
    dem_loaded.clear();
    std::remove(temp_file.c_str());
}

TEST(DemMmapIoTest, StandardStreamFallbackLoad) {
    DemMap dem_orig;
    ASSERT_TRUE(dem_orig.createSyntheticRamp(500.0, 500.0, 10.0, 100.0, 600.0));

    const std::string temp_file = "test_synthetic_ramp.sdem";
    ASSERT_TRUE(dem_orig.saveToFile(temp_file));

    // Nạp qua std::ifstream thường (use_mmap = false)
    DemMap dem_stream;
    ASSERT_TRUE(dem_stream.loadFromFile(temp_file, false));
    EXPECT_TRUE(dem_stream.isLoaded());
    EXPECT_FALSE(dem_stream.isMemoryMapped());

    const auto alt = dem_stream.getElevationBilinear(250.0, 250.0);
    ASSERT_TRUE(alt.has_value());
    EXPECT_NEAR(*alt, 355.1, 1.0);

    dem_stream.clear();
    std::remove(temp_file.c_str());
}


// [M14] Path tieng Viet co dau: Wide API phai mo duoc file (ANSI A-API fail)
TEST(DemMmapIoTest, VietnameseDiacriticPathLoads) {
    survival::platform::initializePlatformHooks();

    DemMap dem_orig;
    ASSERT_TRUE(dem_orig.createSyntheticRamp(100.0, 100.0, 10.0, 1000.0, 1100.0));

    // Ten file co dau tieng Viet — UTF-8 trong nguon
    const std::string path = "b\u1EA3n_\u0111\u1EC3m_th\u1EED.sdem";
    ASSERT_TRUE(dem_orig.saveToFile(path));

    DemMap dem_loaded;
    EXPECT_TRUE(dem_loaded.loadFromFile(path, true));  // mmap qua CreateFileW
    EXPECT_TRUE(dem_loaded.isLoaded());
    const auto z = dem_loaded.getElevationBilinear(50.0, 50.0);
    ASSERT_TRUE(z.has_value());
    std::remove(path.c_str());
}
