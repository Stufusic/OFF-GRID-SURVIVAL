#include <gtest/gtest.h>
#include "survival_core/kinematics/pdr_engine.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_harness/synthetic_generator.hpp"
#include "survival_harness/data_player.hpp"
#include "survival_harness/metrics_collector.hpp"

using namespace survival;
using namespace survival::kinematics;
using namespace survival::harness;
using namespace survival::math;

TEST(PdrIntegrationTest, StraightNorthTrajectoryTracking) {
    // 1. Tạo tập dữ liệu mô phỏng đi thẳng về hướng Bắc trong 20 giây (tốc độ ~ 1.35 m/s -> quãng đường ~ 27m)
    SyntheticConfig config;
    config.trajectory_type = TrajectoryType::kStraightNorth;
    config.duration_s = 20.0;
    config.imu_rate_hz = 100.0;
    config.step_cadence_hz = 1.8;
    config.nominal_stride_length_m = 0.75;
    config.random_seed = 999;

    const auto dataset = SyntheticGenerator::generate(config);

    PdrEngine pdr;
    pdr.reset(dataset.ground_truth_positions.front());

    MetricsCollector collector;
    size_t step_count = 0;

    for (size_t i = 0; i < dataset.sensor_frames.size(); ++i) {
        const auto& frame = dataset.sensor_frames[i];
        const auto opt_step = pdr.processFrame(frame, 0.01);

        if (opt_step.has_value()) {
            step_count++;
            const auto pos = pdr.getCurrentPosition();
            const auto truth = dataset.ground_truth_positions[i];
            const auto cov = pdr.getPositionCovariance2D();

            collector.recordEstimate(
                frame.timestamp_ns,
                pos,
                truth,
                cov[0], // var_e
                cov[3], // var_n
                cov[1], // cov_en
                pdr.getEskf().getHeadingRad(),
                dataset.ground_truth_headings_rad[i]
            );
        }
    }

    // 2. Kiểm tra số lượng bước chân (~36 bước trong 20s)
    EXPECT_GE(step_count, 30U);
    EXPECT_LE(step_count, 42U);

    // 3. Kiểm tra vị trí cuối cùng
    const auto final_pos = pdr.getCurrentPosition();
    const auto final_truth = dataset.ground_truth_positions.back();

    // Đi về hướng Bắc: N phải tăng đáng kể (> 20m), E lệch ít (< 4m)
    EXPECT_GT(final_pos.northing_m, 20.0);
    EXPECT_NEAR(final_pos.easting_m, 0.0, 4.0);

    // 4. Đánh giá sai số qua MetricsCollector
    const auto report = collector.computeReport();
    EXPECT_GT(report.sample_count, 25U);
    EXPECT_LT(report.ce50_m, 5.0);  // Sai số trung vị < 5m
    EXPECT_LT(report.ce95_m, 10.0); // Sai số 95% < 10m
}

TEST(PdrIntegrationTest, TurnRight90TrajectoryTracking) {
    // Đi Bắc 25s rồi rẽ Đông (90 độ)
    SyntheticConfig config;
    config.trajectory_type = TrajectoryType::kTurnRight90;
    config.duration_s = 40.0;
    config.imu_rate_hz = 100.0;
    config.step_cadence_hz = 1.8;
    config.nominal_stride_length_m = 0.75;
    config.random_seed = 888;

    const auto dataset = SyntheticGenerator::generate(config);

    PdrEngine pdr;
    pdr.reset(dataset.ground_truth_positions.front());

    for (size_t i = 0; i < dataset.sensor_frames.size(); ++i) {
        (void)pdr.processFrame(dataset.sensor_frames[i], 0.01);
    }

    const auto final_pos = pdr.getCurrentPosition();
    // Sau khi rẽ Đông: Cả Easting và Northing đều phải dương đáng kể (> 10m)
    EXPECT_GT(final_pos.northing_m, 15.0);
    EXPECT_GT(final_pos.easting_m, 5.0);
}
