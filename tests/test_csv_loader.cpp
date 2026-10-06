#include <gtest/gtest.h>
#include "survival_harness/csv_loader.hpp"
#include "survival_harness/data_player.hpp"
#include "survival_core/kinematics/pdr_engine.hpp"
#include "survival_harness/metrics_collector.hpp"

using namespace survival;
using namespace survival::harness;
using namespace survival::kinematics;

TEST(CsvLoaderTest, ParseBasicSensorStreamWithAutoHeader) {
    const std::string csv_data = 
        "# Sample IMU + Mag + Baro CSV log\n"
        "timestamp_ns, ax, ay, az, gx, gy, gz, mx, my, mz, pressure\n"
        "1000000000, 0.05, 0.10, 9.81, 0.01, -0.02, 0.00, 25.4, -12.1, 40.5, 1013.25\n"
        "1010000000, 0.08, 0.12, 9.79, 0.00, -0.01, 0.01, 25.5, -12.0, 40.4, 1013.20\n"
        "1020000000, -0.02, 0.09, 9.85, -0.01, 0.00, 0.00, 25.3, -12.2, 40.6, 1013.30\n";

    CsvLoaderConfig config;
    const auto opt_dataset = CsvLoader::loadFromString(csv_data, config);

    ASSERT_TRUE(opt_dataset.has_value());
    const auto& dataset = opt_dataset.value();

    EXPECT_EQ(dataset.sensor_frames.size(), 3U);

    // Frame 0
    EXPECT_EQ(dataset.sensor_frames[0].timestamp_ns, 1000000000ULL);
    EXPECT_NEAR(dataset.sensor_frames[0].accel_m_s2[0], 0.05F, 1e-4F);
    EXPECT_NEAR(dataset.sensor_frames[0].accel_m_s2[1], 0.10F, 1e-4F);
    EXPECT_NEAR(dataset.sensor_frames[0].accel_m_s2[2], 9.81F, 1e-4F);
    EXPECT_NEAR(dataset.sensor_frames[0].gyro_rad_s[0], 0.01F, 1e-4F);
    EXPECT_NEAR(dataset.sensor_frames[0].mag_uT[0], 25.4F, 1e-4F);
    EXPECT_NEAR(dataset.sensor_frames[0].pressure_hpa, 1013.25F, 1e-3F);
    EXPECT_TRUE(dataset.sensor_frames[0].has_mag);
    EXPECT_TRUE(dataset.sensor_frames[0].has_baro);
}

TEST(CsvLoaderTest, UnitConversionsAndScaling) {
    const std::string csv_data = 
        "time_ms, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ\n"
        "100.0, 0.0, 0.0, 1.0, 0.0, 0.0, 90.0\n";

    CsvLoaderConfig config;
    config.timestamp_unit = TimestampUnit::kMilliseconds;
    config.accel_unit = AccelerationUnit::kGForce;          // 1g -> ~9.80665 m/s^2
    config.gyro_unit = AngularVelocityUnit::kDegPerSec;     // 90 deg/s -> ~1.570796 rad/s

    const auto opt_dataset = CsvLoader::loadFromString(csv_data, config);
    ASSERT_TRUE(opt_dataset.has_value());
    const auto& dataset = opt_dataset.value();

    ASSERT_EQ(dataset.sensor_frames.size(), 1U);
    EXPECT_EQ(dataset.sensor_frames[0].timestamp_ns, 100000000ULL); // 100 ms = 100,000,000 ns
    EXPECT_NEAR(dataset.sensor_frames[0].accel_m_s2[2], 9.80665F, 1e-3F);
    EXPECT_NEAR(dataset.sensor_frames[0].gyro_rad_s[2], 1.570796F, 1e-3F);
}

TEST(CsvLoaderTest, PipelinePlaybackWithPdrEngine) {
    // Tạo chuỗi dao động Z để kích hoạt phát hiện bước chân
    std::string csv_data = "time_s, ax, ay, az, gx, gy, gz\n";
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.01;
        // Gia tốc dọc trục Z dao động sin tần số 2Hz (bước chân) quanh 9.81 m/s^2
        double az = 9.81 + 3.0 * std::sin(2.0 * kPi * 2.0 * t);
        csv_data += std::to_string(t) + ",0.0,0.0," + std::to_string(az) + ",0.0,0.0,0.0\n";
    }

    CsvLoaderConfig config;
    config.timestamp_unit = TimestampUnit::kSeconds;

    const auto opt_dataset = CsvLoader::loadFromString(csv_data, config);
    ASSERT_TRUE(opt_dataset.has_value());

    DataPlayer player;
    player.loadDataset(opt_dataset.value());

    PdrEngine pdr;
    pdr.reset(EastingNorthingAlt{0.0, 0.0, 0.0});

    size_t steps_detected = 0;
    player.setSensorCallback([&](const SensorFrame& frame) {
        const auto step = pdr.processFrame(frame, 0.01);
        if (step.has_value()) {
            steps_detected++;
        }
    });

    player.playAll();

    EXPECT_TRUE(player.isFinished());
    EXPECT_EQ(player.getTotalFrames(), 200U);
    // 2 giây x 2Hz dao động -> ~3-4 bước chân phát hiện
    EXPECT_GE(steps_detected, 2U);
}

TEST(CsvLoaderTest, RealWorldSensorLoggerFolderReplay) {
    const std::vector<std::string> folders = {
        "C:/Users/KieuKhang/Downloads/2026-09-01_10-57-01_1",
        "C:/Users/KieuKhang/Downloads/2026-09-01_11-24-39_1"
    };

    for (const auto& folder : folders) {
        const auto opt_dataset = CsvLoader::loadSensorLoggerFolder(folder);
        ASSERT_TRUE(opt_dataset.has_value());
        const auto& dataset = opt_dataset.value();

        EXPECT_GT(dataset.sensor_frames.size(), 5000U);

        PdrEngine pdr;
        pdr.reset(EastingNorthingAlt{0.0, 0.0, 0.0});

        size_t detected_steps = 0;
        double max_accel_norm = 0.0;
        double min_accel_norm = 999.0;

        for (size_t i = 0; i < dataset.sensor_frames.size(); ++i) {
            const auto& frame = dataset.sensor_frames[i];
            double dt = (i > 0) ? (static_cast<double>(frame.timestamp_ns - dataset.sensor_frames[i-1].timestamp_ns) * 1e-9) : 0.01;
            if (dt <= 0.0 || dt > 0.1) dt = 0.01;

            double a_norm = std::sqrt(frame.accel_m_s2[0] * frame.accel_m_s2[0] +
                                      frame.accel_m_s2[1] * frame.accel_m_s2[1] +
                                      frame.accel_m_s2[2] * frame.accel_m_s2[2]);
            if (a_norm > max_accel_norm) max_accel_norm = a_norm;
            if (a_norm < min_accel_norm) min_accel_norm = a_norm;

            const auto step = pdr.processFrame(frame, dt);
            if (step.has_value()) {
                detected_steps++;
            }
        }

        const auto final_pos = pdr.getCurrentPosition();
        const double net_dist = std::sqrt(final_pos.easting_m * final_pos.easting_m + final_pos.northing_m * final_pos.northing_m);
        const double duration_s = static_cast<double>(dataset.sensor_frames.back().timestamp_ns - dataset.sensor_frames.front().timestamp_ns) * 1e-9;

        std::cout << "\n================ REAL-WORLD DATASET ANALYSIS REPORT ================\n"
                  << "Dataset               : " << folder << "\n"
                  << "Duration              : " << duration_s << " s (~" << (duration_s / 60.0) << " min)\n"
                  << "Total Sensor Frames   : " << dataset.sensor_frames.size() << " frames (100 Hz)\n"
                  << "Detected Steps Count  : " << detected_steps << " steps (" << (static_cast<double>(detected_steps) / (duration_s > 0 ? duration_s : 1.0)) << " steps/s)\n"
                  << "Accel Norm (min/max)  : " << min_accel_norm << " / " << max_accel_norm << " m/s^2\n"
                  << "Estimated Final Pos   : Easting = " << final_pos.easting_m 
                  << " m, Northing = " << final_pos.northing_m << " m\n"
                  << "Net Displacement      : " << net_dist << " m\n"
                  << "Estimated Step Length : " << (detected_steps > 0 ? (net_dist / static_cast<double>(detected_steps)) : 0.0) << " m (net)\n"
                  << "====================================================================\n";

        EXPECT_GT(detected_steps, 20U);
    }
}



// ============================================================================
// [M11] CSV loader: BOM / row ngan / gtheadingdeg / garbage column
// ============================================================================
TEST(CsvLoaderM11, BomFileTimestampStillMapped) {
    // CSV co BOM (Excel) — truoc fix: cot timestamp khong khop -> bot thoi gian ao
    const std::string csv =
        "\xEF\xBB\xBFtimestamp,ax,ay,az\n"
        "1000000000,0.0,0.0,9.81\n"
        "1010000000,0.0,0.0,9.81\n";
    CsvLoaderConfig cfg{};
    cfg.timestamp_unit = TimestampUnit::kNanoseconds;
    const auto ds = CsvLoader::loadFromString(csv, cfg);
    ASSERT_TRUE(ds.has_value());
    ASSERT_EQ(ds->sensor_frames.size(), 2U);
    EXPECT_EQ(ds->sensor_frames[0].timestamp_ns, 1000000000ULL);
    EXPECT_EQ(ds->sensor_frames[1].timestamp_ns, 1010000000ULL);
}

TEST(CsvLoaderM11, ShortRowDoesNotCrashAndSkipsGt) {
    // Row 2 thieu cot GT (row ngan hon header) — truoc fix: OOB read -> crash
    const std::string csv =
        "timestamp,ax,ay,az,gteasting,gtnorthing\n"
        "1000000000,0.0,0.0,9.81,100.0,200.0\n"
        "1010000000,0.0,0.0,9.81\n";  // thieu 2 cot GT
    CsvLoaderConfig cfg{};
    const auto ds = CsvLoader::loadFromString(csv, cfg);
    ASSERT_TRUE(ds.has_value());
    EXPECT_EQ(ds->sensor_frames.size(), 2U);
    EXPECT_EQ(ds->ground_truth_positions.size(), 1U);  // chi row day du co GT
}

TEST(CsvLoaderM11, GtHeadingDegConvertedToRadians) {
    const std::string csv =
        "timestamp,ax,ay,az,gteasting,gtnorthing,gtheadingdeg\n"
        "1000000000,0.0,0.0,9.81,100.0,200.0,90.0\n";
    CsvLoaderConfig cfg{};
    const auto ds = CsvLoader::loadFromString(csv, cfg);
    ASSERT_TRUE(ds.has_value());
    ASSERT_EQ(ds->ground_truth_headings_rad.size(), 1U);
    EXPECT_NEAR(ds->ground_truth_headings_rad[0], 1.5707963, 1e-4);  // 90 deg = pi/2
}

TEST(CsvLoaderM11, GarbageSensorColumnRejected) {
    // >5% sample parse loi (N/A) -> tu choi dataset thay vi nuot im lang thanh 0.0
    std::string csv = "timestamp,ax,ay,az\n";
    for (int i = 0; i < 30; ++i) {
        csv += std::to_string(1000000000ULL + i * 10000000ULL) + ",N/A,N/A,N/A\n";
    }
    CsvLoaderConfig cfg{};
    const auto ds = CsvLoader::loadFromString(csv, cfg);
    EXPECT_FALSE(ds.has_value());
}
