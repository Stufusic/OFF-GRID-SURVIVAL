#include <gtest/gtest.h>
#include "survival_harness/synthetic_generator.hpp"
#include "survival_harness/data_player.hpp"

using namespace survival;
using namespace survival::harness;

TEST(SyntheticGeneratorTest, MultiRateFrameCountAndTimestamps) {
    SyntheticConfig config;
    config.trajectory_type = TrajectoryType::kStraightNorth;
    config.duration_s = 10.0;
    config.imu_rate_hz = 100.0;
    config.mag_rate_hz = 20.0;
    config.baro_rate_hz = 10.0;
    config.step_cadence_hz = 2.0;

    const auto dataset = SyntheticGenerator::generate(config);

    // 10s * 100Hz = 1000 frames
    EXPECT_EQ(dataset.sensor_frames.size(), 1000U);
    EXPECT_EQ(dataset.ground_truth_positions.size(), 1000U);
    EXPECT_EQ(dataset.ground_truth_headings_rad.size(), 1000U);

    // 10s * 2.0Hz = ~20 steps
    EXPECT_GE(dataset.step_events.size(), 19U);
    EXPECT_LE(dataset.step_events.size(), 21U);

    // Đếm số lượng mẫu Mag (20Hz) và Baro (10Hz)
    size_t mag_count = 0;
    size_t baro_count = 0;
    for (const auto& frame : dataset.sensor_frames) {
        if (frame.has_mag) mag_count++;
        if (frame.has_baro) baro_count++;
    }

    EXPECT_EQ(mag_count, 200U);
    EXPECT_EQ(baro_count, 100U);
}

TEST(DataPlayerTest, SynchronizedReplayExecution) {
    SyntheticConfig config;
    config.duration_s = 5.0;
    const auto dataset = SyntheticGenerator::generate(config);

    DataPlayer player;
    player.loadDataset(dataset);

    size_t received_frames = 0;
    size_t received_steps = 0;
    uint64_t last_frame_ts = 0;
    uint64_t last_step_ts = 0;

    player.setSensorCallback([&](const SensorFrame& frame) {
        received_frames++;
        EXPECT_GE(frame.timestamp_ns, last_frame_ts);
        last_frame_ts = frame.timestamp_ns;
    });

    player.setStepCallback([&](const StepEvent& step) {
        received_steps++;
        EXPECT_GE(step.timestamp_ns, last_step_ts);
        last_step_ts = step.timestamp_ns;
    });

    player.playAll();

    EXPECT_TRUE(player.isFinished());
    EXPECT_EQ(received_frames, dataset.sensor_frames.size());
    EXPECT_EQ(received_steps, dataset.step_events.size());
}
