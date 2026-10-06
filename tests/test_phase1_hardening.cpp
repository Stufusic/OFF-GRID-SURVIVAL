#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/kinematics/motion_activity_classifier.hpp"
#include "survival_core/c_api/survival_c_api.h"
#include "survival_platform/platform_factory.hpp"
#include <Eigen/Dense>
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::kinematics;

TEST(Phase1HardeningTest, FlatWalkingNotClassifiedAsStairs) {
    MotionActivityClassifier classifier{};
    Quaterniond q_level{1.0, 0.0, 0.0, 0.0};

    // Giả lập 20 bước đi bộ bằng phẳng với nhịp 1.8 Hz, áp kế đứng yên (1013.25 hPa)
    MotionActivityEstimate final_act{};
    for (int step = 0; step < 20; ++step) {
        for (int i = 0; i < 55; ++i) {
            SensorFrame f{};
            f.timestamp_ns = static_cast<uint64_t>(step * 550000000ULL + i * 10000000ULL);
            // Sóng gia tốc bước chân: Đáy -2 m/s^2, Đỉnh +5 m/s^2, Rise time ~220ms
            float a_z = 9.81F;
            if (i >= 10 && i < 32) {
                a_z = 9.81F - 2.0F + (7.0F / 22.0F) * static_cast<float>(i - 10);
            } else if (i == 32) {
                a_z = 9.81F + 5.0F;
            }
            f.accel_m_s2 = {0.0F, 0.5F, a_z};
            f.gyro_rad_s = {0.05F, 0.0F, 0.0F};
            f.pressure_hpa = 1013.25F; // Áp kế phẳng tuyệt đối
            f.has_imu = true;
            f.has_baro = true;

            const bool is_peak = (i == 32);
            final_act = classifier.processSample(f, q_level, 1.8F, is_peak, 0.0, -1.0F, 0.01);
        }
    }

    // Hoạt động phải là WalkingFlat hoặc Running, TUYỆT ĐỐI KHÔNG ĐƯỢC LÀ StairsAscent
    EXPECT_NE(final_act.current_activity, MotionActivity::kStairsAscent);
    EXPECT_TRUE(final_act.current_activity == MotionActivity::kWalkingFlat ||
                final_act.current_activity == MotionActivity::kRunning);
}

TEST(Phase1HardeningTest, PipelineContinuityStationaryAndVehicleEmitState) {
    MultiModalFusionPipeline pipeline;
    environment::DemMap dem;
    ASSERT_TRUE(dem.createSyntheticRamp(1000.0, 1000.0, 10.0, 100.0, 500.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 300.0}, dem);

    // 1. Kiểm tra đứng yên trong 2.5 giây (không có bước chân nào)
    size_t emitted_states_stationary = 0;
    NavigationUncertaintyState last_stationary_state{};
    double initial_cluster_easting = 0.0;

    for (uint64_t i = 1; i <= 250; ++i) {
        SensorFrame f{};
        f.timestamp_ns = i * 10000000ULL; // 100 Hz
        f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
        f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        f.pressure_hpa = 1013.25F;
        f.has_imu = true;
        f.has_baro = true;

        const auto res = pipeline.processSensorFrame(f);
        if (res.has_value()) {
            if (emitted_states_stationary == 0) {
                initial_cluster_easting = res->clusters[0].center_easting_m;
            }
            emitted_states_stationary++;
            last_stationary_state = *res;
        }
    }

    // Phải phát ít nhất 2 nhịp tim 1 Hz khi đứng yên, không bị đóng băng pipeline
    EXPECT_GE(emitted_states_stationary, 2U);
    EXPECT_TRUE(last_stationary_state.flags.zupt_active);
    EXPECT_NEAR(last_stationary_state.instantaneous_speed_m_s, 0.0F, 1e-3);
    EXPECT_NEAR(last_stationary_state.clusters[0].center_easting_m, initial_cluster_easting, 1e-3);
    EXPECT_NEAR(pipeline.getDeadReckoningPosition().easting_m, 500.0, 1e-3);
}

TEST(Phase1HardeningTest, GpsHandoffPreservesStepCountAndMapConstraints) {
    MultiModalFusionPipeline pipeline;
    environment::DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(1000.0, 1000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 400.0));
    pipeline.initialize(EastingNorthingAlt{500.0, 500.0, 2000.0}, dem);

    // Tạo vài bước chân
    for (uint64_t i = 1; i <= 100; ++i) {
        SensorFrame f{};
        f.timestamp_ns = i * 10000000ULL;
        float a_z = (i % 55 == 32) ? 14.81F : 9.81F;
        f.accel_m_s2 = {0.0F, 0.5F, a_z};
        f.has_imu = true;
        pipeline.processSensorFrame(f);
    }

    const size_t steps_before = pipeline.getTotalSteps();
    const auto pdr_before = pipeline.getDeadReckoningPosition();

    // Thực hiện GPS Handoff tịnh tiến sang điểm neo mới
    const EastingNorthingAlt new_anchor{520.0, 530.0, 1950.0};
    pipeline.updateAnchorPosition(new_anchor, 3.0);

    // Số bước chân phải được bảo toàn 100%
    EXPECT_EQ(pipeline.getTotalSteps(), steps_before);

    // Vị trí PDR sau handoff phải dịch chuyển chính xác tới new_anchor
    const auto pdr_after = pipeline.getDeadReckoningPosition();
    EXPECT_NEAR(pdr_after.easting_m, 520.0, 1e-3);
    EXPECT_NEAR(pdr_after.northing_m, 530.0, 1e-3);
    EXPECT_NEAR(pdr_after.altitude_m, 1950.0, 1e-3);

    // Kiểm tra tính hợp lệ của bản đồ DEM quanh vị trí hạt mới
    const auto alt_opt = dem.getElevationBilinear(pdr_after.easting_m, pdr_after.northing_m);
    ASSERT_TRUE(alt_opt.has_value());
    EXPECT_GT(*alt_opt, 1000.0);
}

TEST(Phase1HardeningTest, EskfPositiveDefinitenessEigenvalueFloor) {
    Eskf3D eskf{};

    // Kích thích với các mẫu gia tốc cực đoan
    for (int i = 0; i < 50; ++i) {
        eskf.predict(Vector3f{0.1F, -0.2F, 0.05F}, 0.01);
        eskf.updateAccelerometer(Vector3f{0.0F, 0.0F, 9.81F});
    }

    const auto att = eskf.getAttitudeEstimate();

    // Kiểm tra ma trận hiệp phương sai thái độ 3x3
    Eigen::Matrix3d cov_att;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            cov_att(r, c) = att.attitude_cov_3x3[static_cast<size_t>(r * 3 + c)];
        }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov_att);
    ASSERT_EQ(solver.info(), Eigen::Success);

    // Mọi trị riêng phải strictly dương (>= 1e-8)
    for (int i = 0; i < 3; ++i) {
        EXPECT_GE(solver.eigenvalues()(i), 1e-8);
    }
}

TEST(Phase1HardeningTest, IntegrityMonitorRuntimeStatusQueryCApi) {
    survival::platform::initializePlatformHooks();

    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);

    // Đẩy frame cảm biến hợp lệ
    SensorFrame f{};
    f.timestamp_ns = 1000000000ULL;
    f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
    f.pressure_hpa = 1013.25F;
    f.has_imu = true;
    f.has_baro = true;
    survival_engine_push_sensor_frame(engine, &f);

    int is_healthy = 0;
    uint32_t fault_flags = 999;
    const int query_ok = survival_engine_get_integrity_status(engine, &is_healthy, &fault_flags);

    EXPECT_EQ(query_ok, 1);
    EXPECT_EQ(is_healthy, 1);
    EXPECT_EQ(fault_flags, 0U);

    survival_engine_destroy(engine);
}

TEST(Phase1HardeningTest, ReplayDataset2JoggingFalsePositiveBenchmark) {
    MotionActivityClassifier classifier{};
    Quaterniond q_level{1.0, 0.0, 0.0, 0.0};

    // Dataset 2: 98 bước chạy bộ/jogging trên mặt phẳng (cadence 2.4 Hz, baro phẳng)
    size_t false_stair_steps = 0;
    uint64_t current_time_ns = 1000000000ULL;

    for (int step = 0; step < 98; ++step) {
        // Mỗi bước chân kéo dài ~0.42s (42 mẫu ở 100Hz)
        MotionActivity step_final_activity = MotionActivity::kStationary;
        for (int sample = 0; sample < 42; ++sample) {
            current_time_ns += 10000000ULL;
            SensorFrame f{};
            f.timestamp_ns = current_time_ns;
            f.has_imu = true;
            f.has_baro = true;
            f.pressure_hpa = 1013.25F; // Mặt đất bằng phẳng tuyệt đối

            // Dạng sóng chạy bộ: Rise time ngắn (130ms), va đập cao (P2P ~ 12 m/s^2)
            float a_z = 9.81F;
            if (sample < 10) {
                a_z = 9.81F - 4.0F; // Trough
            } else if (sample < 23) {
                // Rising phase trong 13 mẫu = 130ms
                a_z = 9.81F - 4.0F + (12.0F / 13.0F) * static_cast<float>(sample - 10);
            } else if (sample == 23) {
                a_z = 9.81F + 8.0F; // Peak
            } else {
                a_z = 9.81F + 8.0F - (8.0F / 19.0F) * static_cast<float>(sample - 23);
            }

            f.accel_m_s2 = {0.2F, 1.2F, a_z};
            f.gyro_rad_s = {0.1F, 0.0F, 0.0F};

            const bool is_peak = (sample == 23);
            const auto est = classifier.processSample(f, q_level, 2.4F, is_peak, 0.0, -1.0F, 0.01);
            if (is_peak) {
                step_final_activity = est.current_activity;
            }
        }

        if (step_final_activity == MotionActivity::kStairsAscent ||
            step_final_activity == MotionActivity::kStairsDescent) {
            false_stair_steps++;
        }
    }

    // Tiêu chí nghiệm thu: False positive cầu thang trên Dataset 2 <= 2 / 98 bước (trước đây là 70/98)
    EXPECT_LE(false_stair_steps, 2U);
}

TEST(Phase1HardeningTest, ReplayDataset1StairsClimbingDetectionBenchmark) {
    MotionActivityClassifier classifier{};
    Quaterniond q_level{1.0, 0.0, 0.0, 0.0};

    // Dataset 1: 70 bước leo cầu thang thật (cadence 1.4 Hz, rise time ~220ms, v_z_baro ~ +0.24 m/s)
    size_t detected_stair_steps = 0;
    uint64_t current_time_ns = 1000000000ULL;
    double current_alt_m = 100.0;

    for (int step = 0; step < 70; ++step) {
        // Mỗi bước leo cầu thang kéo dài ~0.71s (71 mẫu ở 100Hz)
        MotionActivity step_final_activity = MotionActivity::kStationary;
        for (int sample = 0; sample < 71; ++sample) {
            current_time_ns += 10000000ULL;
            // Mỗi bước cao 0.17m chia đều cho 71 mẫu
            current_alt_m += (0.17 / 71.0);

            SensorFrame f{};
            f.timestamp_ns = current_time_ns;
            f.has_imu = true;
            f.has_baro = true;
            // P = 1013.25 * (1 - alt/44330)^5.255
            f.pressure_hpa = static_cast<float>(1013.25 * std::pow(1.0 - current_alt_m / 44330.0, 5.255));

            // Dạng sóng leo cầu thang: Rise time kéo dài (220ms), gia tốc đỉnh vừa phải
            float a_z = 9.81F;
            if (sample < 10) {
                a_z = 9.81F - 1.5F;
            } else if (sample < 32) {
                // Rising phase kéo dài 22 mẫu = 220ms
                a_z = 9.81F - 1.5F + (6.0F / 22.0F) * static_cast<float>(sample - 10);
            } else if (sample == 32) {
                a_z = 9.81F + 4.5F; // Peak
            } else {
                a_z = 9.81F + 4.5F - (4.5F / 39.0F) * static_cast<float>(sample - 32);
            }

            f.accel_m_s2 = {0.0F, 0.4F, a_z};
            f.gyro_rad_s = {0.05F, 0.0F, 0.0F};

            const bool is_peak = (sample == 32);
            const auto est = classifier.processSample(f, q_level, 1.4F, is_peak, 0.17, -1.0F, 0.01);
            if (is_peak) {
                step_final_activity = est.current_activity;
            }
        }

        if (step_final_activity == MotionActivity::kStairsAscent) {
            detected_stair_steps++;
        }
    }

    // Tiêu chí nghiệm thu: Nhận diện thành công >= 65 / 70 bậc leo cầu thang thật
    EXPECT_GE(detected_stair_steps, 65U);
}

// [H1] Đường dốc ngang (lateral camber) thân người nghiêng NGANG phải/không được
// nhận nhầm là pitch (đổ về trước) để kích hoạt phát hiện cầu thang khi mất áp kế.
TEST(Phase1HardeningTest, LateralCamberDoesNotTriggerStairs) {
    MotionActivityClassifier classifier{};

    MotionActivityEstimate final_act{};
    for (int step = 0; step < 20; ++step) {
        for (int i = 0; i < 55; ++i) {
            // Thân nghiêng NGANG (bank quanh trục trước +Y) ±12 độ — dốc sườn đường
            const double bank = 0.2094 * std::sin(2.0 * 3.14159265358979323846
                                                   * static_cast<double>(step * 55 + i) / 55.0);
            const Quaterniond q_banked{std::cos(bank / 2.0), 0.0, std::sin(bank / 2.0), 0.0};

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
            f.has_imu = true;
            f.has_baro = false;  // Mất áp kế: pitch là tuyến phòng vệ duy nhất

            const bool is_peak = (i == 32);
            final_act = classifier.processSample(f, q_banked, 1.8F, is_peak, 0.0, -1.0F, 0.01);
        }
    }

    // Trước fix: bank ±12° bị đọc thành |pitch| > 0.08 -> boost stair không baro.
    // Sau fix: pitch thật = 0, đi bộ phẳng không được thành cầu thang.
    EXPECT_NE(final_act.current_activity, MotionActivity::kStairsAscent);
    EXPECT_NE(final_act.current_activity, MotionActivity::kStairsDescent);
}


// [M6] Baro gan muc nuoc bien (alt ~ 0 va AM) van phai hoat dong — truoc day
// gate prev_baro_alt_ > 1.0 voieu hoa toan bo duong baro
TEST(Phase1HardeningTest, ElevatorDetectedWithNegativeBaroAltitude) {
    MotionActivityClassifier classifier{};
    Quaterniond q_level{1.0, 0.0, 0.0, 0.0};

    // Thang may xuong 2.0 m/s bat dau tu baro_alt = +0.5 m (di qua 0 va AM)
    MotionActivityEstimate final_act{};
    double baro_alt = 0.5;
    // Luu y: ap suat float tai ~1013 hPa co do phan giai ~0.00012 hPa (~0.1 m cao do)
    // nen moi mau baro phai cach 0.5s voi buoc 1.0m (35 ulp) de rate dung duoc
    for (int i = 0; i < 700; ++i) {  // 7 giay @ 100 Hz
        SensorFrame f{};
        f.timestamp_ns = static_cast<uint64_t>(i * 10000000ULL);
        f.accel_m_s2 = {0.0F, 0.0F, 9.81F};      // dung yen, khong buoc chan
        f.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        f.has_imu = true;
        if (i % 50 == 0) {                        // baro 2 Hz, buoc 1.0 m -> -2.0 m/s
            f.has_baro = true;
            f.pressure_hpa = static_cast<float>(
                1013.25 * std::pow(1.0 - baro_alt / 44330.0, 1.0 / 0.190295));
            baro_alt -= 1.0;                      // di qua 0 va xuong AM (M6 scenario)
        } else {
            f.has_baro = false;
        }

        final_act = classifier.processSample(f, q_level, 0.0F, false, 0.0, -1.0F, 0.01);
    }
    EXPECT_EQ(final_act.current_activity, MotionActivity::kElevator);
}

