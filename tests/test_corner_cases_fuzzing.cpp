#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/map_manager.hpp"
#include "survival_core/safety/safety_auditor.hpp"
#include "survival_core/safety/resource_governor.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include "survival_core/math/statistics.hpp"
#include "survival_core/integrity/integrity_guard.hpp"
#include <limits>
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::environment;
using namespace survival::safety;
using namespace survival::comms;

TEST(CornerCasesFuzzingTest, NanAndInfinitySensorFramesRejection) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(1000.0, 1000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 400.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1500.0}, dem);

    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();

    // 1. Frame với gia tốc NaN
    SensorFrame f_nan{};
    f_nan.timestamp_ns = 1000000000ULL;
    f_nan.accel_m_s2 = {static_cast<float>(kNan), 0.0F, 9.81F};
    f_nan.gyro_rad_s = {0.0F, static_cast<float>(kInf), 0.0F};
    f_nan.has_imu = true;

    const auto res_nan = pipeline.processSensorFrame(f_nan);
    (void)res_nan;

    // Vị trí PDR không được bị nhiễm độc NaN
    const auto pdr = pipeline.getDeadReckoningPosition();
    EXPECT_FALSE(std::isnan(pdr.easting_m));
    EXPECT_FALSE(std::isnan(pdr.northing_m));
    EXPECT_FALSE(std::isnan(pdr.altitude_m));
}

TEST(CornerCasesFuzzingTest, ExtremeCoordinateDemQueries) {
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 500.0, 1500.0));

    // 1. Tọa độ cực xa ngoài không gian (1 triệu km)
    EXPECT_FALSE(dem.isInside(1e9, 1e9));
    EXPECT_EQ(dem.getElevationBilinear(1e9, 1e9), std::nullopt);

    // 2. Tọa độ âm cực lớn
    EXPECT_FALSE(dem.isInside(-1e9, -1e9));
    EXPECT_EQ(dem.getElevationBilinear(-1e9, -1e9), std::nullopt);

    // 3. Gradient tại điểm ngoài bản đồ phải trả về is_valid = false
    const auto grad = dem.getGradient(1e9, 1e9);
    EXPECT_FALSE(grad.is_valid);
}

TEST(CornerCasesFuzzingTest, CollinearCovarianceEllipseCalculation) {
    // Trường hợp ma trận hiệp phương sai suy biến 1 chiều: var_e = 0, var_n = 100, cov = 0
    const auto ell_1d = math::computeUncertaintyEllipse(0.0, 100.0, 0.0, 5.9915);
    EXPECT_GE(ell_1d.semi_minor_m, 0.0);
    EXPECT_GT(ell_1d.semi_major_m, 0.0);
    EXPECT_FALSE(std::isnan(ell_1d.semi_major_m));
    EXPECT_FALSE(std::isnan(ell_1d.semi_minor_m));
    EXPECT_FALSE(std::isnan(ell_1d.orientation_rad));

    // Trường hợp phương sai bằng 0 tuyệt đối cả 2 chiều
    const auto ell_zero = math::computeUncertaintyEllipse(0.0, 0.0, 0.0, 5.9915);
    EXPECT_GE(ell_zero.semi_major_m, 0.0);
    EXPECT_GE(ell_zero.semi_minor_m, 0.0);
}

TEST(CornerCasesFuzzingTest, Packet160BitflipCorruptionSweep) {
    const auto valid_packet = Packet160Serializer::encode(
        1700000000, 21.0285, 105.8542, 1200.0, 25.0, 15.0, 45.0, 0x01, 88, 1
    );

    // Đảo từng bit trong toàn bộ 160 bits (20 bytes * 8 bits = 160 ca thử nghiệm)
    for (size_t byte_idx = 0; byte_idx < 20; ++byte_idx) {
        for (uint8_t bit_idx = 0; bit_idx < 8; ++bit_idx) {
            auto corrupted = valid_packet;
            corrupted[byte_idx] ^= (1U << bit_idx);

            DecodedRescuePacket decoded{};
            const bool ok = Packet160Serializer::decode(corrupted, decoded);
            // Mọi bit bị đảo đều phải bị phát hiện và từ chối qua kiểm tra CRC
            EXPECT_FALSE(ok);
            EXPECT_FALSE(decoded.is_crc_valid);
        }
    }
}
