#include <gtest/gtest.h>
#include "renderer/ribbon_renderer.hpp"

using namespace survival;
using namespace survival::gui;

TEST(RibbonMeshTest, GenerateRibbonVerticesAndIndicesExactCount) {
    std::vector<EastingNorthingAlt> waypoints = {
        {0.0, 0.0, 1000.0},
        {0.0, 50.0, 1002.0},
        {50.0, 50.0, 1005.0},
        {50.0, 100.0, 1010.0}
    };

    const size_t N = waypoints.size();
    const auto geom = RibbonRenderer::generateRibbonStrip(waypoints, 4.0);

    // 1. Số đỉnh phải là 2 * N = 8 đỉnh
    EXPECT_EQ(geom.vertices.size(), N * 2);

    // 2. Số chỉ số tam giác phải là 6 * (N - 1) = 18 indices
    EXPECT_EQ(geom.indices.size(), (N - 1) * 6);

    // 3. Toàn bộ index phải nằm trong dải [0, 2*N - 1]
    for (const uint16_t idx : geom.indices) {
        EXPECT_LT(idx, static_cast<uint16_t>(N * 2));
    }
}
