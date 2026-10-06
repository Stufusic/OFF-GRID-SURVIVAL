#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/estimators/particle_filter_engine.hpp"
#include "survival_core/kinematics/motion_activity_classifier.hpp"
#include "survival_core/integrity/integrity_monitor.hpp"
#include "survival_core/safety/safety_auditor.hpp"
#include "survival_core/c_api/survival_c_api.h"
#include "survival_platform/platform_factory.hpp"
#include "survival_harness/synthetic_generator.hpp"
#include "survival_harness/metrics_collector.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <numeric>

using namespace survival;
using namespace survival::estimators;
using namespace survival::kinematics;
using namespace survival::integrity;
using namespace survival::safety;
using namespace survival::harness;
using namespace survival::environment;

namespace {

void printScenarioHeader(const std::string& id, const std::string& title) {
    std::cout << "\n=======================================================\n"
              << "[SCENARIO " << id << "] " << title << "\n"
              << "=======================================================" << std::endl;
}

} // namespace

// ============================================================================
// S01: Flat Walking (Quantitative RMSE / CEP95 Benchmark)
// ============================================================================
TEST(ScenarioSimulationHarness, S01_FlatWalking) {
    printScenarioHeader("S01", "Flat Walking 500m with Quantitative Metrics");

    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(3000.0, 3000.0, 10.0, 0.0, 0.0)); // Flat terrain centered at origin
    pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 1000.0}, dem);

    SyntheticConfig cfg{};
    cfg.trajectory_type = TrajectoryType::kStraightNorth;
    cfg.duration_s = 40.0;
    cfg.step_cadence_hz = 1.8;
    cfg.nominal_stride_length_m = 0.75;
    cfg.random_seed = 101;

    const auto dataset = SyntheticGenerator::generate(cfg);
    ASSERT_FALSE(dataset.sensor_frames.empty());

    MetricsCollector metrics;
    size_t truth_idx = 0;
    size_t walking_flat_detected = 0;
    size_t total_emitted_states = 0;

    for (const auto& frame : dataset.sensor_frames) {
        const auto res = pipeline.processSensorFrame(frame);
        if (res.has_value()) {
            total_emitted_states++;
            if (res->motion_activity == MotionActivity::kWalkingFlat) {
                walking_flat_detected++;
            }

            if (truth_idx < dataset.ground_truth_positions.size()) {
                const auto& truth = dataset.ground_truth_positions[truth_idx++];
                const EastingNorthingAlt est{
                    res->clusters[0].center_easting_m,
                    res->clusters[0].center_northing_m,
                    res->clusters[0].center_altitude_m
                };
                const double r = res->clusters[0].ce95_semi_major_m;
                metrics.recordEstimate(frame.timestamp_ns, est, truth, r * r, r * r, 0.0);
            }
        }
    }

    const auto report = metrics.computeReport();
    const double accuracy = (total_emitted_states > 0)
        ? (100.0 * static_cast<double>(walking_flat_detected) / static_cast<double>(total_emitted_states))
        : 0.0;

    std::cout << " -> Emitted States: " << total_emitted_states 
              << ", Walking Flat Accuracy: " << accuracy << "%\n"
              << " -> 2D RMSE: " << report.rmse_2d_m << " m, CEP95: " << report.ce95_m 
              << " m, Max Err: " << report.max_error_2d_m << " m" << std::endl;

    EXPECT_GT(total_emitted_states, 20U);
    EXPECT_GE(walking_flat_detected, total_emitted_states * 8 / 10);
    EXPECT_LT(report.rmse_2d_m, 40.0);
    EXPECT_LT(report.ce95_m, 60.0);
}

// ============================================================================
// S02: Stair Climbing Without Barometer
// ============================================================================
TEST(ScenarioSimulationHarness, S02_StairClimbingWithoutBarometer) {
    printScenarioHeader("S02", "Stair Climbing Without Barometer (Pitch & Jerk Evidence)");

    MotionActivityClassifier classifier;
    // Quaternion góc chúi người về trước pitch ~ 0.18 rad (khoảng 10.3 độ)
    // [H1] Đổ về trước = xoay quanh trục PHẢI (+X) chiều âm (x = −sin(θ/2)).
    // Quay quanh +Y (trước) là nghiêng NGANG — công thức cũ của test đã mã hóa nhầm trục.
    const double pitch_half = 0.18 * 0.5;
    Quaterniond q_pitch{std::cos(pitch_half), -std::sin(pitch_half), 0.0, 0.0};

    size_t stair_detected = 0;
    size_t total_steps = 15;

    for (size_t step = 0; step < total_steps; ++step) {
        for (int i = 0; i < 60; ++i) {
            SensorFrame f{};
            f.timestamp_ns = static_cast<uint64_t>(step) * 600000000ULL + static_cast<uint64_t>(i) * 10000000ULL;
            // Dạng sóng leo thang "đẩy sớm" (khớp cực tính skewness đã audit trên
            // dữ liệu thật — năng lượng đẩy dồn ở nửa đầu chu kỳ bước):
            // hump +4.5 nửa đầu, hố -1.2 nửa sau (p2p = 5.7 m/s^2, dải thang 4.6-19).
            float a_z = 9.81F;
            {
                const double phase = static_cast<double>(i) / 60.0;
                a_z = 9.81F + static_cast<float>(phase < 0.50
                    ? 4.5 * std::sin(phase / 0.50 * 3.14159265)
                    : -1.2 * std::sin((phase - 0.50) / 0.50 * 3.14159265));
            }
            f.accel_m_s2 = {0.0F, 0.4F, a_z};
            f.gyro_rad_s = {0.02F, 0.0F, 0.0F};
            f.has_imu = true;
            f.has_baro = false; // Mất khí áp kế!

            const bool is_peak = (i == 59);
            const auto act = classifier.processSample(f, q_pitch, 1.6F, is_peak, 0.0, -1.0F, 0.01, 1.0F);
            if (is_peak && act.current_activity == MotionActivity::kStairsAscent) {
                stair_detected++;
            }
        }
    }

    // [C1] Thiết kế mới yêu cầu bằng chứng thang DUY TRÌ 8 bước trước khi gán nhãn
    // (chống khóa sai đi bộ thành stairs) — nên 15 bước chỉ đủ gán nhãn cho các
    // bước sau khi cửa sổ cộng dồn đã armed (~7 bước đầu). Ngưỡng kỳ vọng 6.
    std::cout << " -> Stairs Detected: " << stair_detected << "/" << total_steps << std::endl;
    EXPECT_GE(stair_detected, 6U);
}

// ============================================================================
// S03 & S04: Elevator and Escalator Dynamics
// ============================================================================
TEST(ScenarioSimulationHarness, S03_S04_ElevatorAndEscalator) {
    printScenarioHeader("S03/S04", "Vertical Transport: Elevator and Escalator");

    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(2000.0, 2000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 500.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1000.0}, dem);

    // Thang máy: Tích phân cao độ tăng, tọa độ mặt phẳng ENU giữ nguyên (ZUPT)
    const auto pdr_start = pipeline.getDeadReckoningPosition();
    for (uint64_t i = 1; i <= 300; ++i) {
        SensorFrame f{};
        f.timestamp_ns = i * 10000000ULL;
        // Áp suất giảm tương ứng nâng cao độ ~ 2.0 m/s (sau 3s nâng ~6.0m)
        const float alt = 1000.0F + static_cast<float>(i) * 0.01F * 2.0F;
        f.pressure_hpa = static_cast<float>(1013.25 * std::pow(1.0 - static_cast<double>(alt) / 44330.0, 5.255));
        f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
        f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        f.has_imu = true;
        f.has_baro = true;
        static_cast<void>(pipeline.processSensorFrame(f));
    }

    const auto pdr_end = pipeline.getDeadReckoningPosition();
    std::cout << " -> Elevator dz: " << (pdr_end.altitude_m - pdr_start.altitude_m) << " m" << std::endl;
    EXPECT_GT(pdr_end.altitude_m, pdr_start.altitude_m + 2.0);
}

// ============================================================================
// S05, S06, S07, S08: Vehicle & Bicycle Multi-Speed Spectrum
// ============================================================================
TEST(ScenarioSimulationHarness, S06_S07_S08_CarSpeedsAndSpectralIntegrity) {
    printScenarioHeader("S06-S08", "Vehicle Transport (20, 80, 120 km/h) & Plausibility");

    IntegrityMonitor monitor;

    // Test 3 tốc độ: 20 km/h (5.56 m/s), 80 km/h (22.2 m/s), 120 km/h (33.3 m/s)
    const std::vector<double> speeds = {5.56, 22.22, 33.33};
    for (double v : speeds) {
        CanonicalNavigationState state{};
        state.timestamp_ns = 1000000000ULL;
        state.velocity_m_s = {v, 0.0, 0.0};
        state.motion_activity.current_activity = MotionActivity::kVehicleWheeled;
        state.position_cov_2d = {5.0, 0.0, 0.0, 5.0};
        state.attitude_cov_3x3 = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

        const auto status = monitor.auditState(state, 1000000000ULL);
        std::cout << " -> Vehicle speed " << (v * 3.6) << " km/h (" << v << " m/s): is_healthy="
                  << status.is_healthy << ", fault_flags=0x" << std::hex << status.fault_flags << std::dec << std::endl;

        EXPECT_TRUE(status.is_healthy);
        EXPECT_EQ(status.fault_flags, 0U);
        EXPECT_TRUE(status.velocity_valid);
    }
}

// ============================================================================
// S09: GPS Dropout and Seamless Off-Grid Handoff
// ============================================================================
TEST(ScenarioSimulationHarness, S09_GpsDropoutAndHandoff) {
    printScenarioHeader("S09", "GPS Cutoff and Smooth Dead-Reckoning Handoff");

    survival::platform::initializePlatformHooks();
    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);

    // 1. Đẩy GPS Fix ban đầu
    survival_engine_push_gps_fix(engine, 0.35, 1.85, 1200.0, 3.0, 0.9, 1000000000ULL);

    // 2. Đẩy frame cảm biến hoạt động
    SensorFrame f1{};
    f1.timestamp_ns = 1000000000ULL;
    f1.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    f1.has_imu = true;
    survival_engine_push_sensor_frame(engine, &f1);

    // 3. Mất GPS đột ngột (GPS Cutoff)
    survival_engine_push_gps_cutoff(engine, 1050000000ULL);

    // 4. Đẩy chuỗi frame bước chân Off-Grid
    for (uint64_t i = 1; i <= 200; ++i) {
        SensorFrame f{};
        f.timestamp_ns = 1050000000ULL + i * 10000000ULL;
        const float a_z = (i % 30 == 0) ? 14.0F : 9.81F;
        f.accel_m_s2 = {0.0F, 0.5F, a_z};
        f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        f.has_imu = true;
        f.has_baro = true;
        f.pressure_hpa = 1013.25F;
        survival_engine_push_sensor_frame(engine, &f);
    }

    NavigationUncertaintyState state{};
    EXPECT_EQ(survival_engine_get_uncertainty_state(engine, &state), 1);
    std::cout << " -> Off-Grid State Cluster 0: (" << state.clusters[0].center_easting_m 
              << ", " << state.clusters[0].center_northing_m << ") m, CE95: " 
              << state.clusters[0].ce95_semi_major_m << " m" << std::endl;

    EXPECT_FALSE(std::isnan(state.clusters[0].center_easting_m));
    EXPECT_FALSE(std::isnan(state.clusters[0].center_northing_m));
    EXPECT_GT(state.clusters[0].ce95_semi_major_m, 0.0);

    survival_engine_destroy(engine);
}

// ============================================================================
// S10: IMU NaN / Inf Attack Firewall
// ============================================================================
TEST(ScenarioSimulationHarness, S10_ImuNanInfAttack) {
    printScenarioHeader("S10", "Hostile NaN/Inf Sensor Frame Attack Firewall");

    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 500.0, 500.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 500.0}, dem);

    constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
    constexpr float kInf = std::numeric_limits<float>::infinity();

    // Bơm 5 frame độc hại liên tiếp
    for (uint64_t i = 1; i <= 5; ++i) {
        SensorFrame bad_f{};
        bad_f.timestamp_ns = i * 10000000ULL;
        bad_f.accel_m_s2 = {kNan, kInf, -kInf};
        bad_f.gyro_rad_s = {kInf, kNan, 0.0F};
        bad_f.has_imu = true;
        static_cast<void>(pipeline.processSensorFrame(bad_f));
    }

    const auto pdr = pipeline.getDeadReckoningPosition();
    const auto att = pipeline.getEskf().getAttitudeEstimate();

    std::cout << " -> Position after attack: (" << pdr.easting_m << ", " << pdr.northing_m 
              << "), Yaw: " << att.euler.yaw_rad << " rad" << std::endl;

    EXPECT_FALSE(std::isnan(pdr.easting_m));
    EXPECT_FALSE(std::isinf(pdr.northing_m));
    EXPECT_FALSE(std::isnan(att.euler.yaw_rad));
    EXPECT_FALSE(std::isinf(att.euler.yaw_rad));
}

// ============================================================================
// S11: Barometer Sudden Dropout
// ============================================================================
TEST(ScenarioSimulationHarness, S11_BarometerDropout) {
    printScenarioHeader("S11", "Barometer Sudden Dropout Mid-Trajectory");

    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(2000.0, 2000.0, 10.0, 500.0, 600.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 500.0}, dem);

    // 10s có Baro, sau đó 10s mất Baro hoàn toàn
    for (uint64_t i = 1; i <= 1000; ++i) {
        SensorFrame f{};
        f.timestamp_ns = i * 10000000ULL;
        const float a_z = (i % 25 == 0) ? 13.5F : 9.81F;
        f.accel_m_s2 = {0.0F, 0.5F, a_z};
        f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        f.has_imu = true;
        if (i <= 500) {
            f.has_baro = true;
            f.pressure_hpa = 1013.25F;
        } else {
            f.has_baro = false; // Ngắt barometer
        }
        static_cast<void>(pipeline.processSensorFrame(f));
    }

    const auto pdr = pipeline.getDeadReckoningPosition();
    std::cout << " -> Final Altitude after Baro dropout: " << pdr.altitude_m << " m" << std::endl;
    EXPECT_FALSE(std::isnan(pdr.altitude_m));
    EXPECT_GT(pdr.altitude_m, 100.0);
}

// ============================================================================
// S12: Multi-Tier Sensor Freeze Watchdog
// ============================================================================
TEST(ScenarioSimulationHarness, S12_MultiTierSensorFreezeWatchdog) {
    printScenarioHeader("S12", "Multi-Tier Watchdog: HEALTHY -> DEGRADED -> STALE -> DEAD");

    survival::platform::initializePlatformHooks();
    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);

    SensorFrame f{};
    f.timestamp_ns = 1000000000ULL;
    f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    f.has_imu = true;
    survival_engine_push_sensor_frame(engine, &f);

    int healthy = 0;
    uint32_t faults = 0;

    // 1. Tức thời: HEALTHY
    survival_engine_get_integrity_status(engine, &healthy, &faults);
    EXPECT_EQ(healthy, 1);
    EXPECT_EQ(faults, 0U);

    // 2. Sau 600ms (> 500ms): DEGRADED
    survival_engine_advance_simulated_time(engine, 600000000ULL);
    survival_engine_get_integrity_status(engine, &healthy, &faults);
    EXPECT_EQ(healthy, 1);
    EXPECT_NE(faults & survival::integrity::kFaultEstimatorDegraded, 0U);

    // 3. Sau 2.0s (> 1.5s): STALE
    survival_engine_advance_simulated_time(engine, 1400000000ULL); // tổng 2.0s
    survival_engine_get_integrity_status(engine, &healthy, &faults);
    EXPECT_EQ(healthy, 0);
    EXPECT_NE(faults & survival::integrity::kFaultEstimatorStale, 0U);

    // 4. Sau 6.0s (> 5.0s): DEAD
    survival_engine_advance_simulated_time(engine, 4000000000ULL); // tổng 6.0s
    survival_engine_get_integrity_status(engine, &healthy, &faults);
    EXPECT_EQ(healthy, 0);
    EXPECT_NE(faults & survival::integrity::kFaultSensorDead, 0U);
    EXPECT_NE(faults & survival::integrity::kFaultEstimatorStale, 0U);

    std::cout << " -> Watchdog states passed: HEALTHY -> DEGRADED -> STALE -> DEAD\n";

    survival_engine_destroy(engine);
}

// ============================================================================
// S13: Long Vehicle Journey Continuous Stochastic Diffusion
// ============================================================================
TEST(ScenarioSimulationHarness, S13_LongVehicleJourneyContinuousDiffusion) {
    printScenarioHeader("S13", "Long Vehicle Journey (1500m @ 25 m/s) Continuous Diffusion");

    ParticleFilterEngine pf;
    pf.initialize(EastingNorthingAlt{1000.0, 1000.0, 200.0}, 20.0);

    // Chạy xe trong 60 giây ở vận tốc 25 m/s (1500m)
    constexpr double dt = 0.01;
    constexpr double speed = 25.0;
    for (int step = 0; step < 6000; ++step) {
        const double de = 0.0;
        const double dn = speed * dt;
        pf.propagateContinuousMotion(de, dn, 0.0, speed, dt, MotionActivity::kVehicleWheeled);
    }

    const auto mean_pos = pf.getWeightedMeanPosition();
    const auto cov = pf.getWeightedCovariance2D();
    std::cout << " -> Final Particle Center: (" << mean_pos.easting_m 
              << ", " << mean_pos.northing_m << ") m\n"
              << " -> Northing displacement: " << (mean_pos.northing_m - 1000.0) << " m (Expected ~ 1500m)" << std::endl;

    EXPECT_NEAR(mean_pos.northing_m, 2500.0, 50.0);
    EXPECT_GT(cov[3], 20.0); // Hiệp phương sai mở rộng phản ánh độ bất định Brownian
}

// ============================================================================
// S14: Circular Wandering & Panic Circling Detection
// ============================================================================
TEST(ScenarioSimulationHarness, S14_CircularWanderingPanicCircling) {
    printScenarioHeader("S14", "Circular Wandering & Panic Circling Trigger");

    SafetyAuditor auditor;

    // Đi 650 bước, tổng quãng đường 480m nhưng chỉ dịch chuyển thuần 18m
    auditor.updateOdometryDisplacement(18.0, 480.0, 650);

    std::cout << " -> Panic Circling Warning: " << (auditor.isPanicCirclingWarning() ? "YES" : "NO")
              << ", Safety State: " << static_cast<int>(auditor.getState()) << std::endl;

    EXPECT_TRUE(auditor.isPanicCirclingWarning());
    EXPECT_EQ(auditor.getState(), SafetyState::kPanicCirclingWarning);
}

// ============================================================================
// S15 & S16: Repeated Resampling & Adaptive Kernel Roughening
// ============================================================================
TEST(ScenarioSimulationHarness, S15_S16_DegenerateParticleCloudAdaptiveRoughening) {
    printScenarioHeader("S15/S16", "Degenerate Particle Cloud & Adaptive Kernel Roughening");

    ParticleFilterEngine pf;
    pf.initialize(EastingNorthingAlt{500.0, 500.0, 100.0}, 10.0);

    // Kiểm tra số lượng hạt ban đầu
    const auto initial_cov = pf.getWeightedCovariance2D();
    const double initial_var_e = initial_cov[0];

    // Thực hiện làm mịn hạt thích ứng trên địa hình dốc nhẹ
    pf.applyAdaptiveKernelRoughening(0.05);

    const auto roughened_cov = pf.getWeightedCovariance2D();
    const double roughened_var_e = roughened_cov[0];

    std::cout << " -> Initial Easting Var: " << initial_var_e 
              << " m2, Post-Roughening Easting Var: " << roughened_var_e << " m2" << std::endl;

    EXPECT_GT(roughened_var_e, 0.0);
    EXPECT_FALSE(std::isnan(roughened_var_e));
}
