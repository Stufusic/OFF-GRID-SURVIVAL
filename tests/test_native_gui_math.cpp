#include <gtest/gtest.h>
#include "egl/fps_governor.hpp"
#include "renderer/ribbon_renderer.hpp"
#include <cmath>

using namespace survival::gui;
using namespace survival;

TEST(NativeGuiTest, FpsGovernorAdaptiveStateTransitions) {
    FpsGovernor gov;

    // 1. Đứng yên, màn hình bật, pin 90% -> 1 FPS
    gov.updateMotionState(false, 0.0, 90.0, true);
    EXPECT_EQ(gov.getActivityState(), RenderActivityState::kStationary);
    EXPECT_DOUBLE_EQ(gov.getTargetFps(), 1.0);

    // 2. Di chuyển hoặc xoay góc hướng -> 60 FPS
    gov.updateMotionState(true, 0.05, 90.0, true);
    EXPECT_EQ(gov.getActivityState(), RenderActivityState::kMovingOrRotating);
    EXPECT_DOUBLE_EQ(gov.getTargetFps(), 60.0);

    // 3. Pin yếu 3% -> 0.1 FPS (10s/frame)
    gov.updateMotionState(true, 0.05, 3.0, true);
    EXPECT_EQ(gov.getActivityState(), RenderActivityState::kBatteryCritical);
    EXPECT_DOUBLE_EQ(gov.getTargetFps(), 0.1);

    // 4. Tắt màn hình (Pocket Mode) -> 0 FPS
    gov.updateMotionState(true, 0.05, 90.0, false);
    EXPECT_EQ(gov.getActivityState(), RenderActivityState::kScreenOff);
    EXPECT_DOUBLE_EQ(gov.getTargetFps(), 0.0);
    EXPECT_FALSE(gov.shouldRenderFrame(1000000000ULL));
}

TEST(NativeGuiTest, RibbonRendererEllipseMesh64Vertices) {
    // Sinh lưới Elip tại tâm (100, 200), bán trục a = 40m, b = 20m, xoay 45 độ
    const auto geom = RibbonRenderer::generateEllipseMesh(100.0, 200.0, 40.0, 20.0, 0.785398);

    // 64 đỉnh vành + 1 đỉnh tâm = 65 đỉnh
    EXPECT_EQ(geom.vertices.size(), 65U);
    // 64 tam giác * 3 = 192 indices
    EXPECT_EQ(geom.indices.size(), 192U);

    // Đỉnh tâm phải ở (100, 200)
    EXPECT_FLOAT_EQ(geom.vertices[0].x, 100.0F);
    EXPECT_FLOAT_EQ(geom.vertices[0].y, 200.0F);
}

TEST(NativeGuiTest, RibbonRendererTriangleStripGeneration) {
    std::vector<EastingNorthingAlt> waypoints = {
        {0.0, 0.0, 100.0},
        {0.0, 10.0, 100.0},
        {10.0, 10.0, 100.0}
    };

    const auto ribbon = RibbonRenderer::generateRibbonStrip(waypoints, 4.0); // Rộng 4m (±2m)

    // 3 waypoints * 2 đỉnh (trái/phải) = 6 đỉnh
    EXPECT_EQ(ribbon.vertices.size(), 6U);
}
