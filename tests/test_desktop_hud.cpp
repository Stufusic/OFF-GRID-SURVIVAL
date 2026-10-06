// ============================================================================
// [U3] DESKTOP HUD MATH TESTS — toán thuần của dashboard (không cần cửa sổ)
// ============================================================================
#include <gtest/gtest.h>
#include "../../desktop/src/hud_math.hpp"

using namespace desktop;

TEST(DesktopHudMath, MapViewRoundTripWorldScreen) {
    MapView view;
    view.center_east_m = 1000.0;
    view.center_north_m = 1000.0;
    view.pixels_per_m = 0.5;
    constexpr double kVpW = 800.0, kVpH = 600.0;

    // Tâm màn hình = tâm world
    EXPECT_FLOAT_EQ(view.screenX(1000.0, kVpW), kVpW / 2.0F);
    EXPECT_FLOAT_EQ(view.screenY(1000.0, kVpH), kVpH / 2.0F);

    // Round-trip world -> screen -> world
    const float sx = view.screenX(1120.0, kVpW);
    const float sy = view.screenY(940.0, kVpH);
    EXPECT_NEAR(view.worldEast(sx, kVpW), 1120.0, 0.01);
    EXPECT_NEAR(view.worldNorth(sy, kVpH), 940.0, 0.01);

    // North hướng LÊN màn hình (Y-flip): north lớn hơn -> screenY nhỏ hơn
    EXPECT_LT(view.screenY(1100.0, kVpH), view.screenY(900.0, kVpH));
}

TEST(DesktopHudMath, MapViewPanPixels) {
    MapView view;
    view.center_east_m = 1000.0;
    view.center_north_m = 1000.0;
    view.pixels_per_m = 1.0;

    // Kéo chuột sang phải 100px -> world pan sang TRÁI 100m (kéo nội dung)
    view.panPixels(100.0, 0.0);
    EXPECT_NEAR(view.center_east_m, 900.0, 0.01);

    // Kéo xuống 50px -> North tăng 50m (kéo nội dung xuống)
    view.panPixels(0.0, 50.0);
    EXPECT_NEAR(view.center_north_m, 1050.0, 0.01);
}

TEST(DesktopHudMath, MapViewZoomKeepsCursorPoint) {
    MapView view;
    view.center_east_m = 1000.0;
    view.center_north_m = 1000.0;
    view.pixels_per_m = 0.5;
    constexpr double kVpW = 800.0, kVpH = 600.0;

    const double e_cursor = view.worldEast(200.0, kVpW);
    const double n_cursor = view.worldNorth(400.0, kVpH);

    view.zoomAt(200.0, 400.0, kVpW, kVpH, 2.0);

    EXPECT_NEAR(view.pixels_per_m, 1.0, 0.001);
    // Điểm world dưới con trỏ không đổi
    EXPECT_NEAR(view.worldEast(200.0, kVpW), e_cursor, 0.01);
    EXPECT_NEAR(view.worldNorth(400.0, kVpH), n_cursor, 0.01);
}

TEST(DesktopHudMath, MapViewFitTo) {
    MapView view;
    view.pixels_per_m = 0.02;
    std::vector<std::pair<double, double>> pts = {
        {900.0, 900.0}, {1100.0, 1100.0}};
    view.fitTo(pts, 800.0, 600.0, 40.0);

    // Span 200m phải nằm gọn trong 720px -> scale ~3.0 nhưng bị kẹp tối đa 8.0
    EXPECT_GT(view.pixels_per_m, 1.0);
    EXPECT_NEAR(view.center_east_m, 1000.0, 0.5);
    EXPECT_NEAR(view.center_north_m, 1000.0, 0.5);
}

TEST(DesktopHudMath, EllipseOutlineCountAndRotation) {
    // Heading 0 (Bắc): major trục dọc theo North
    const auto outline = ellipseOutline(1000.0, 1000.0, 50.0, 10.0, 0.0);
    EXPECT_EQ(outline.size(), 64U);
    // Điểm t=0 (i=0) = tâm + major theo hướng Bắc (east giữ nguyên)
    EXPECT_NEAR(outline[0].first, 1000.0, 0.01);   // east
    EXPECT_NEAR(outline[0].second, 1050.0, 0.01);  // north = 1000 + 50
    // Điểm i=16 (t=90°) = minor theo East
    EXPECT_NEAR(outline[16].first, 1010.0, 0.01);
    EXPECT_NEAR(outline[16].second, 1000.0, 0.01);
}

TEST(DesktopHudMath, RingBufferFifoAndCopyOut) {
    RingBuffer<int, 4> rb;
    rb.push(1); rb.push(2); rb.push(3);
    int out[4] = {};
    EXPECT_EQ(rb.copyOut(out, 4), 3U);
    EXPECT_EQ(out[0], 1); EXPECT_EQ(out[1], 2); EXPECT_EQ(out[2], 3);

    rb.push(4); rb.push(5);  // ghi đè vòng: giữ {2,3,4,5}
    EXPECT_EQ(rb.size(), 4U);
    EXPECT_EQ(rb.copyOut(out, 4), 4U);
    EXPECT_EQ(out[0], 2); EXPECT_EQ(out[3], 5);
    EXPECT_EQ(rb.latestOr(0), 5);
}

TEST(DesktopHudMath, GeoOriginConversion) {
    const GeoOrigin origin{21.0285, 105.8542};
    const auto [lat, lon] = origin.enToLatLon(0.0, 0.0);
    EXPECT_NEAR(lat, 21.0285, 1e-9);
    EXPECT_NEAR(lon, 105.8542, 1e-9);

    // 1000m Bắc ≈ +0.009°
    const auto [lat2, lon2] = origin.enToLatLon(0.0, 1000.0);
    EXPECT_NEAR(lat2 - 21.0285, 1000.0 / 111320.0, 1e-6);
    // Kinh độ đổi theo cos(lat)
    const auto [lat3, lon3] = origin.enToLatLon(111320.0 * std::cos(21.0285 * 3.14159265358979323846 / 180.0), 0.0);
    EXPECT_NEAR(lon3 - 105.8542, 1.0, 0.01);
}

TEST(DesktopHudMath, HexDumpFormat) {
    const unsigned char data[3] = {0x0A, 0xFF, 0x01};
    EXPECT_EQ(toHexDump(data, 3), "0A FF 01");
    EXPECT_EQ(toHexDump(data, 0), "");
}
