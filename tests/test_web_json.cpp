// ============================================================================
// [W4] WEB DASHBOARD JSON WRITER TESTS
// ============================================================================
#include <gtest/gtest.h>
#include "../../desktop/src/web_server.hpp"

using namespace desktop;

TEST(WebJson, EscapeHandlesQuotesAndControlChars) {
    EXPECT_EQ(jsonEscape("plain"), "plain");
    EXPECT_EQ(jsonEscape("has \"quote\""), "has \\\"quote\\\"");
    EXPECT_EQ(jsonEscape("back\\slash"), "back\\\\slash");
    EXPECT_EQ(jsonEscape("line\nbreak"), "line\\nbreak");
    EXPECT_EQ(jsonEscape(std::string("ctrl\x01x")), "ctrl\\u0001x");
}

TEST(WebJson, StateHasAllTopLevelFields) {
    WebState s{};
    s.gns = 0.789;
    s.ce95_m = 12.5;
    s.total_steps = 42;
    s.activity = 3;
    s.safety_state = 2;
    s.packet_hex = "AA BB";
    s.packet_sms = "SOS test";
    s.timestamp_ms = 1234567;

    const std::string json = buildStateJson(s);

    // Trường bắt buộc cho web UI
    EXPECT_NE(json.find("\"gns\":0.789"), std::string::npos);
    EXPECT_NE(json.find("\"ce95_m\":12.5"), std::string::npos);
    EXPECT_NE(json.find("\"steps\":42"), std::string::npos);
    EXPECT_NE(json.find("\"activity\":3"), std::string::npos);
    EXPECT_NE(json.find("\"safety_state\":2"), std::string::npos);
    EXPECT_NE(json.find("\"packet_hex\":\"AA BB\""), std::string::npos);
    EXPECT_NE(json.find("\"packet_sms\":\"SOS test\""), std::string::npos);
    EXPECT_NE(json.find("\"timestamp_ms\":1234567"), std::string::npos);
    EXPECT_NE(json.find("\"flags\""), std::string::npos);
    EXPECT_NE(json.find("\"clusters\":[],"), std::string::npos);
    EXPECT_NE(json.find("\"particles\":[]"), std::string::npos);
}

TEST(WebJson, EscapesMaliciousPacketSms) {
    WebState s{};
    s.packet_sms = "x\"y\nz";
    const std::string json = buildStateJson(s);
    // JSON vẫn parse-được về mặt cấu trúc: chuỗi bị escape, không có \n thô
    EXPECT_EQ(json.find("x\"y"), std::string::npos);
    EXPECT_NE(json.find("x\\\"y\\nz"), std::string::npos);
}

TEST(WebJson, ParticlesOnlyAliveAndTrailAndClusters) {
    WebState s{};
    s.particles = {{1.0F, 2.0F, true}, {3.0F, 4.0F, false}, {5.0F, 6.0F, true}};
    s.trail = {{100.0, 200.0}, {300.0, 400.0}};
    WebCluster c{10.0, 20.0, 30.0, 15.0, 0.5};
    s.clusters.push_back(c);

    const std::string json = buildStateJson(s);
    // Hạt chết bị loại
    EXPECT_EQ(json.find("[3,4]"), std::string::npos);
    EXPECT_NE(json.find("[1.000,2.000]"), std::string::npos);  // fixed 3 decimals
    EXPECT_NE(json.find("[5.000,6.000]"), std::string::npos);
    // Trail + cluster đầy đủ
    EXPECT_NE(json.find("[100.000,200.000]"), std::string::npos);
    EXPECT_NE(json.find("\"a\":30"), std::string::npos);
    EXPECT_NE(json.find("\"b\":15"), std::string::npos);
    EXPECT_NE(json.find("\"h\":0.5"), std::string::npos);
}

TEST(WebJson, SeriesArraysPresent) {
    WebState s{};
    s.series.gns = {0.5F, 0.6F, 0.7F};
    s.series.speed = {1.0F};
    const std::string json = buildStateJson(s);
    EXPECT_NE(json.find("\"series_gns\":[0.500,0.600,0.700]"), std::string::npos);
    EXPECT_NE(json.find("\"series_speed\":[1.000]"), std::string::npos);
    EXPECT_NE(json.find("\"series_vertical\":[]"), std::string::npos);
}
