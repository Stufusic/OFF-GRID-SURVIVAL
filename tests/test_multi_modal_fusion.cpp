#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_harness/synthetic_generator.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/integrity/integrity_monitor.hpp"
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::harness;
using namespace survival::environment;

TEST(MultiModalFusionTest, TimeJitterAndDropProtection) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1000.0, 1500.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1200.0}, dem);

    // Frame 1: Chuẩn (t = 1.0s)
    SensorFrame f1{};
    f1.timestamp_ns = 1000000000ULL;
    f1.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    const auto r1 = pipeline.processSensorFrame(f1);
    (void)r1;

    // Frame 2: Thời gian bị ngược (t = 0.5s < 1.0s) -> Fallback an toàn dt = 0.01s
    SensorFrame f2{};
    f2.timestamp_ns = 500000000ULL;
    f2.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    const auto r2 = pipeline.processSensorFrame(f2);
    (void)r2;

    // Frame 3: Nhảy vọt thời gian lớn (t = 10.0s, delta = 9.5s > 0.5s) -> Fallback an toàn dt = 0.01s
    SensorFrame f3{};
    f3.timestamp_ns = 10000000000ULL;
    f3.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    const auto r3 = pipeline.processSensorFrame(f3);
    (void)r3;

    const auto pdr_pos = pipeline.getDeadReckoningPosition();
    EXPECT_FALSE(std::isnan(pdr_pos.easting_m));
    EXPECT_FALSE(std::isnan(pdr_pos.northing_m));
    EXPECT_FALSE(std::isnan(pdr_pos.altitude_m));
}

TEST(MultiModalFusionTest, EndToEndMultiRateWalkingStream) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(2000.0, 2000.0, 10.0, 1000.0, 1000.0, 2000.0, 1000.0, 600.0));
    pipeline.initialize(EastingNorthingAlt{1000.0, 800.0, 1300.0}, dem);

    SyntheticConfig synth_cfg{};
    synth_cfg.duration_s = 10.0;
    synth_cfg.step_cadence_hz = 1.8;
    synth_cfg.nominal_stride_length_m = 0.75;
    synth_cfg.random_seed = 42;
    synth_cfg.trajectory_type = TrajectoryType::kStraightNorth;

    const auto dataset = SyntheticGenerator::generate(synth_cfg);
    ASSERT_GT(dataset.sensor_frames.size(), 500U);

    size_t state_updates = 0;
    std::optional<NavigationUncertaintyState> last_state;

    for (const auto& frame : dataset.sensor_frames) {
        auto res = pipeline.processSensorFrame(frame);
        if (res.has_value()) {
            state_updates++;
            last_state = res;
        }
    }

    // 10 giây đi bộ ở 1.8 Hz phải phát hiện khoảng 16-20 bước chân
    EXPECT_GE(state_updates, 10U);
    EXPECT_LE(state_updates, 25U);

    // Quỹ đạo PDR phải tiến về phía Bắc
    const auto pdr_pos = pipeline.getDeadReckoningPosition();
    EXPECT_GT(pdr_pos.northing_m, 0.0);

    // Trạng thái bất định cuối cùng phải hợp lệ
    ASSERT_TRUE(last_state.has_value());
    EXPECT_GE(last_state->active_clusters_count, 1U);
    EXPECT_GT(last_state->clusters[0].ce95_semi_major_m, 0.0);
    EXPECT_GT(last_state->global_navigation_score, 0.0);
}

// [H2] position_wgs84 phải là DEGREES (enuToLatLon trả về radians — kiểm tra hệ số đổi)
TEST(CanonicalStateWgs84, DegreesNotRadians) {
    // Origin WGS-84: 21.0285°B, 105.8542°Đ (Hà Nội) -> radians
    constexpr double kOriginLatDeg = 21.0285;
    constexpr double kOriginLonDeg = 105.8542;
    constexpr double kOriginLatRad = kOriginLatDeg * (3.14159265358979323846 / 180.0);
    constexpr double kOriginLonRad = kOriginLonDeg * (3.14159265358979323846 / 180.0);

    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1000.0, 1100.0,
                                        kOriginLatRad, kOriginLonRad));
    // Anchor ENU tại DEM origin (0,0)
    pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 1050.0}, dem);

    SensorFrame f{};
    f.timestamp_ns = 1000000000ULL;
    f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    (void)pipeline.processSensorFrame(f);

    const auto canon = pipeline.getLatestCanonicalState();

    // Trước fix: trường *_deg chứa radians (~0.36) -> test fail.
    // Sau fix: đúng độ. 500 m lệch ENU gốc => 0 ở đây, chỉ kiểm tra datum + đơn vị.
    EXPECT_NEAR(canon.position_wgs84.latitude_deg, kOriginLatDeg, 1e-3);
    EXPECT_NEAR(canon.position_wgs84.longitude_deg, kOriginLonDeg, 1e-3);

    // Invariant 2 (No Null-Island Egress): không bao giờ (0, 0) khi DEM có datum thật
    EXPECT_NE(canon.position_wgs84.latitude_deg, 0.0);
    EXPECT_NE(canon.position_wgs84.longitude_deg, 0.0);

    // Kiểm tra phép chiếu xuôi: +500 m về hướng Bắc phải tăng vĩ độ đúng tỷ lệ quả cầu
    constexpr double kEarthRadiusMeanM = 6371000.0;
    const double expected_lat_deg = kOriginLatDeg + (500.0 / kEarthRadiusMeanM) * (180.0 / 3.14159265358979323846);
    pipeline.updateAnchorPosition(EastingNorthingAlt{0.0, 500.0, 1050.0}, 2.0);
    // Canonical state chỉ refresh theo nhịp emit (>= 1 Hz fallback) -> frame cách mốc emit trước > 1s
    SensorFrame f2{};
    f2.timestamp_ns = 2100000000ULL;  // last_output = 1.0s -> delta 1.1s >= 1s
    f2.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    (void)pipeline.processSensorFrame(f2);
    const auto canon3 = pipeline.getLatestCanonicalState();
    EXPECT_NEAR(canon3.position_wgs84.latitude_deg, expected_lat_deg, 1e-4);
}


// ============================================================================
// [S1] Regression tests cho cac fix M1/M2/M4/M5
// ============================================================================

// [M1] baro_v_z phai tinh dung cap mau baro 10 Hz — truoc day spike 10x/0 xen ke
TEST(M1BaroRate, StableVerticalSpeedAtTenHzBaro) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1000.0, 1100.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1050.0}, dem);

    // Di bo dang song buoc chan + baro 10 Hz tang deu 0.5 m/s
    // p(alt) = 1013.25 * (1 - alt/44330)^(1/0.190295)
    double baro_alt_m = 50.0;
    size_t emit_count = 0;
    size_t vz_in_range = 0;
    size_t vz_total_after_warmup = 0;
    for (int step = 0; step < 20; ++step) {
        for (int i = 0; i < 55; ++i) {
            SensorFrame f{};
            f.timestamp_ns = static_cast<uint64_t>(step * 550000000ULL + i * 10000000ULL);
            float a_z = 9.81F;
            if (i >= 10 && i < 32) {
                a_z = 9.81F - 2.0F + (7.0F / 22.0F) * static_cast<float>(i - 10);
            } else if (i == 32) {
                a_z = 9.81F + 5.0F;
            }
            f.accel_m_s2 = {0.0F, 0.5F, a_z};
            f.gyro_rad_s = {0.05F, 0.0F, 0.0F};
            // Baro chi co mau moi 10 frame (10 Hz trong luong 100 Hz)
            if (i % 10 == 0) {
                f.has_baro = true;
                f.pressure_hpa = static_cast<float>(
                    1013.25 * std::pow(1.0 - baro_alt_m / 44330.0, 1.0 / 0.190295));
                baro_alt_m += 0.05;   // 0.5 m/s * 100 ms (baro 10 Hz trong luong 100 Hz)
            } else {
                f.has_baro = false;
            }
            f.has_imu = true;

            auto res = pipeline.processSensorFrame(f);
            if (res.has_value()) {
                emit_count++;
                if (step >= 8) {  // bo qua khoi dong (LPF vz can ~2s hoi tu)
                    vz_total_after_warmup++;
                    const float vz = res->vertical_speed_m_s;
                    if (vz > 0.1F && vz < 1.2F) {
                        vz_in_range++;
                    }
                }
            }
        }
    }
    ASSERT_GT(vz_total_after_warmup, 10U);
    // Truoc fix: vz dao dong 0 <-> 5x toc do that -> ty le trong khoang thap
    EXPECT_GT(static_cast<double>(vz_in_range) / static_cast<double>(vz_total_after_warmup), 0.8);
}

// [M4] Watchdog staleness noi bo: frame nhay thoi gian -> kFaultEstimatorStale
TEST(M4Watchdog, EstimatorStaleFiresOnEmissionGap) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1000.0, 1100.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1050.0}, dem);

    // Phat 1 Hz binh thuong: t = 1s, 2s
    for (uint64_t t_ns : {1000000000ULL, 2000000000ULL}) {
        SensorFrame f{};
        f.timestamp_ns = t_ns;
        f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
        f.has_imu = true;
        f.has_baro = true;
        f.pressure_hpa = 1000.0F;
        (void)pipeline.processSensorFrame(f);
    }
    const auto status_before = pipeline.getLatestIntegrityStatus();
    EXPECT_TRUE(status_before.is_healthy);

    // Nhay thang: frame ke tiep tai t = 5.5s (cach moc phat cuoi 3.5s > 1.5s)
    SensorFrame f{};
    f.timestamp_ns = 5500000000ULL;
    f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    f.has_imu = true;
    f.has_baro = true;
    f.pressure_hpa = 1000.0F;
    (void)pipeline.processSensorFrame(f);

    const auto status = pipeline.getLatestIntegrityStatus();
    EXPECT_FALSE(status.is_healthy);
    EXPECT_NE(status.fault_flags & survival::integrity::kFaultEstimatorStale, 0U);
}

// [M5] Canonical state phai duoc gan day du
TEST(M5Canonical, FullyPopulatedCanonicalState) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1000.0, 1100.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1050.0}, dem);

    for (uint64_t t_ns : {1000000000ULL, 2000000000ULL}) {
        SensorFrame f{};
        f.timestamp_ns = t_ns;
        f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
        f.has_imu = true;
        f.has_baro = true;
        f.pressure_hpa = 1000.0F;
        (void)pipeline.processSensorFrame(f);
    }

    const auto canon = pipeline.getLatestCanonicalState();
    // Truoc fix: nav_mode mac dinh kGpsAided, position_source mac dinh kGps, ce95 2.45
    EXPECT_NE(canon.nav_mode, NavigationMode::kGpsAided);
    EXPECT_EQ(canon.position_source, PositionSource::kPdrTrn);
    EXPECT_GT(canon.ce95_m, 3.0);   // tu phan tan hat that (khong con gia tri default 2.45)
    EXPECT_NE(canon.integrity.fault_flags, 0xFFFFFFFFU);  // integrity gan dung struct
}


// [R1] Di bo phang KHONG baro (song p2p manh 7 m/s^2) — khong duoc cong do cao ao
// +0.17 m/buoc nua (Bug 3: flat-guard cap p_stair + pipeline gate FSM-confirmed)
TEST(R1StairGate, FlatWalkNoBaroNoAltitudeDrift) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1000.0, 1100.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1050.0}, dem);

    const double alt0 = pipeline.getDeadReckoningPosition().altitude_m;
    for (int step = 0; step < 20; ++step) {
        for (int i = 0; i < 55; ++i) {
            SensorFrame f{};
            f.timestamp_ns = static_cast<uint64_t>(step * 550000000ULL + i * 10000000ULL);
            float a_z = 9.81F;
            if (i >= 10 && i < 32) {
                a_z = 9.81F - 2.0F + (7.0F / 22.0F) * static_cast<float>(i - 10);
            } else if (i == 32) {
                a_z = 9.81F + 5.0F;
            }
            f.accel_m_s2 = {0.0F, 0.5F, a_z};
            f.gyro_rad_s = {0.0F, 0.0F, 0.0F};  // gyro 0: gyro bias gia lap se lam
            f.has_imu = true;                    // troi pitch ESKF sau 11s
            f.has_baro = false;  // Mat ap ke — dung truong hop Bug 3

            (void)pipeline.processSensorFrame(f);
        }
    }
    const double alt1 = pipeline.getDeadReckoningPosition().altitude_m;
    // Truoc fix: +0.17 m/buoc x 20 = +3.4 m do cao ao
    EXPECT_LT(alt1 - alt0, 0.3);
    EXPECT_GT(alt1 - alt0, -0.3);
}


// [B2] Bias ap ke QNH: DEM PHANG (khong the duoi theo cao do) + baro lech +30 m
// -> bias EMA hoi tu ve +30, PF khong bi keo xuong sai cao do
TEST(B2QnhBias, ConvergesToBaroOffsetOnFlatDem) {
    MultiModalFusionPipeline pipeline;
    DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 1050.0, 1050.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 1050.0}, dem);

    const double baro_alt = 1080.0;
    const float baro_hpa = static_cast<float>(
        1013.25 * std::pow(1.0 - baro_alt / 44330.0, 1.0 / 0.190295));
    for (int step = 0; step < 218; ++step) {  // 218 buoc x 0.55s ~ 120 s
        for (int i = 0; i < 55; ++i) {
            SensorFrame f{};
            f.timestamp_ns = static_cast<uint64_t>(step * 550000000ULL + i * 10000000ULL);
            float a_z = 9.81F;
            if (i >= 10 && i < 32) {
                a_z = 9.81F - 2.0F + (7.0F / 22.0F) * static_cast<float>(i - 10);
            } else if (i == 32) {
                a_z = 9.81F + 5.0F;
            }
            f.accel_m_s2 = {0.0F, 0.5F, a_z};
            f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
            f.has_imu = true;
            f.has_baro = true;
            f.pressure_hpa = baro_hpa;
            (void)pipeline.processSensorFrame(f);
        }
    }
    const double bias = pipeline.getBaroBiasM();
    EXPECT_GT(bias, 15.0);
    EXPECT_LT(bias, 45.0);
}
