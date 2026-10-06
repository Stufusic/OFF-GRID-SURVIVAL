#include "survival_harness/benchmark_runner.hpp"
#include "survival_core/math/statistics.hpp"
#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace survival::harness {

BenchmarkRunner::BenchmarkRunner(const BenchmarkConfig& config) noexcept
    : config_(config)
{
}

BenchmarkReport BenchmarkRunner::run() noexcept {
    BenchmarkReport report{};
    report.mode = config_.mode;
    report.scenario = config_.scenario;
    report.total_runs = config_.num_monte_carlo_runs;

    // 1. Khởi tạo bản đồ DEM
    environment::DemMap dem;
    dem.createSyntheticCone(
        config_.dem_width_m,
        config_.dem_height_m,
        10.0,
        config_.dem_width_m * 0.5,
        config_.dem_height_m * 0.5,
        config_.dem_peak_alt_m,
        config_.dem_base_alt_m,
        800.0
    );

    std::vector<double> run_ce50_list;
    std::vector<double> run_ce95_list;
    std::vector<double> run_nees_list;
    std::vector<double> fast_loop_latencies_us;
    std::vector<double> pf_step_latencies_ms;
    double global_max_nis = 0.0;

    run_ce50_list.reserve(config_.num_monte_carlo_runs);
    run_ce95_list.reserve(config_.num_monte_carlo_runs);
    run_nees_list.reserve(config_.num_monte_carlo_runs);

    for (size_t run_idx = 0; run_idx < config_.num_monte_carlo_runs; ++run_idx) {
        // A. Khóa hạt giống kép cho mỗi lượt lặp Monte Carlo
        const uint32_t current_env_seed = config_.environment_seed + static_cast<uint32_t>(run_idx * 100);
        const uint32_t current_est_seed = config_.estimator_seed + static_cast<uint32_t>(run_idx * 100);

        SyntheticConfig synth_cfg{};
        synth_cfg.duration_s = static_cast<double>(config_.dataset_step_count) * 0.60;
        synth_cfg.step_cadence_hz = 1.8;
        synth_cfg.nominal_stride_length_m = 0.75;
        synth_cfg.random_seed = current_env_seed;
        synth_cfg.trajectory_type = TrajectoryType::kStraightNorth;

        const auto synthetic_data = SyntheticGenerator::generate(synth_cfg);

        FailureConfig fail_cfg{};
        fail_cfg.scenario = config_.scenario;
        fail_cfg.environment_seed = current_env_seed;

        FailureInjector injector(fail_cfg);
        MetricsCollector metrics;

        estimators::PipelineConfig pipe_cfg{};
        pipe_cfg.random_seed = current_est_seed;
        pipe_cfg.pf_config.num_particles = (config_.mode == AlgorithmMode::kPurePDR || config_.mode == AlgorithmMode::kEskfPDR) ? 100 : 1000;
        pipe_cfg.initial_baro_alt_m = config_.dem_base_alt_m;

        estimators::MultiModalFusionPipeline pipeline(pipe_cfg);
        const EastingNorthingAlt init_pos{0.0, 0.0, config_.dem_base_alt_m};
        pipeline.initialize(init_pos, dem);

        size_t gt_idx = 0;

        for (const auto& raw_frame : synthetic_data.sensor_frames) {
            SensorFrame frame = raw_frame;
            injector.injectSensorFrame(frame);

            // Đo độ trễ Fast Loop
            const auto t0 = std::chrono::steady_clock::now();
            const auto uncertainty_opt = pipeline.processSensorFrame(frame);
            const auto t1 = std::chrono::steady_clock::now();

            const double dt_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            fast_loop_latencies_us.push_back(dt_us);

            const double current_nis = pipeline.getLatestNis();
            if (current_nis > global_max_nis) {
                global_max_nis = current_nis;
            }

            if (uncertainty_opt.has_value()) {
                // Nhịp xử lý bước chân & Particle Filter: thời gian frame thực thi bộ lọc hạt
                const double dt_ms = dt_us * 1e-3;
                pf_step_latencies_ms.push_back(dt_ms);

                // Đối chiếu với Ground Truth
                if (gt_idx < synthetic_data.ground_truth_positions.size()) {
                    const auto& gt = synthetic_data.ground_truth_positions[gt_idx];
                    
                    if (config_.mode == AlgorithmMode::kPurePDR || config_.mode == AlgorithmMode::kEskfPDR) {
                        const auto pdr_pos = pipeline.getDeadReckoningPosition();
                        // [H3] Hiệp phương sai PDR thật (m^2) thay cho hardcode 10.0/10.0
                        const auto pdr_cov = pipeline.getDeadReckoningCovariance();
                        metrics.recordEstimate(
                            frame.timestamp_ns,
                            pdr_pos,
                            gt,
                            pdr_cov[0], pdr_cov[3], pdr_cov[1]
                        );
                    } else {
                        const auto& u_state = *uncertainty_opt;
                        const auto& c0 = u_state.clusters[0];
                        const EastingNorthingAlt est_pos{c0.center_easting_m, c0.center_northing_m, c0.center_altitude_m};
                        // [H3] Elip 95% (bán trục [m]) phải đổi về PHƯƠNG SAI [m^2] trước khi tính NEES:
                        // var = (a / sqrt(chi2_95))^2 — truyền bán trục thẳng làm P phình ~6-12x, NEES ≈ 0.
                        const auto cov = survival::math::ellipseToCov2D(
                            c0.ce95_semi_major_m, c0.ce95_semi_minor_m, c0.ellipse_heading_rad);
                        metrics.recordEstimate(
                            frame.timestamp_ns,
                            est_pos,
                            gt,
                            cov[0], cov[3], cov[1]
                        );
                    }
                    gt_idx++;
                }
            }
        }

        const auto report_metrics = metrics.computeReport();
        run_ce50_list.push_back(report_metrics.ce50_m);
        run_ce95_list.push_back(report_metrics.ce95_m);
        run_nees_list.push_back(report_metrics.mean_nees_2d);
    }

    // 2. Tổng hợp báo cáo thống kê
    if (!run_ce50_list.empty()) {
        report.mean_ce50_m = std::accumulate(run_ce50_list.begin(), run_ce50_list.end(), 0.0) / run_ce50_list.size();
        report.mean_ce95_m = std::accumulate(run_ce95_list.begin(), run_ce95_list.end(), 0.0) / run_ce95_list.size();
        report.mean_nees = std::accumulate(run_nees_list.begin(), run_nees_list.end(), 0.0) / run_nees_list.size();
    }

    if (!fast_loop_latencies_us.empty()) {
        report.avg_fast_loop_cpu_us = std::accumulate(fast_loop_latencies_us.begin(), fast_loop_latencies_us.end(), 0.0) / fast_loop_latencies_us.size();
    }

    if (!pf_step_latencies_ms.empty()) {
        report.avg_particle_step_cpu_ms = std::accumulate(pf_step_latencies_ms.begin(), pf_step_latencies_ms.end(), 0.0) / pf_step_latencies_ms.size();
    }

    report.max_nis = global_max_nis;
    report.p_loss = (report.mean_ce95_m > 100.0) ? 0.30 : 0.05;

    // 3. Đánh giá tiêu chuẩn nghiệm thu khoa học
    report.passes_m1_magnetic_criteria = (report.mean_ce95_m <= 100.0);
    // [C4] M2 dùng tiêu chí TƯƠNG ĐỐI theo đúng đặc tả header ("CE95 tăng <= 20%
    // so với M0"): chạy baseline M0 nội bộ ghép cặp cùng seed/số lượt. Chỉnh define
    // tuyệt đối 120m cũ lệch đặc tả (rvharness.md).
    if (config_.scenario == ScenarioType::kM2_BarometricDrift) {
        BenchmarkConfig baseline_cfg = config_;
        baseline_cfg.scenario = ScenarioType::kM0_Nominal;
        BenchmarkRunner baseline_runner(baseline_cfg);
        const auto baseline_report = baseline_runner.run();
        report.m0_baseline_ce95_m = baseline_report.mean_ce95_m;
        report.passes_m2_baro_drift_criteria =
            (report.mean_ce95_m <= 1.20 * report.m0_baseline_ce95_m) &&
            (report.mean_ce95_m <= 150.0);  // trần an toàn tuyệt đối đề phòng
    } else {
        report.passes_m2_baro_drift_criteria = (report.mean_ce95_m <= 120.0);
    }
    report.passes_latency_criteria = (report.avg_fast_loop_cpu_us <= 150.0 && report.avg_particle_step_cpu_ms <= 10.0);

    return report;
}

} // namespace survival::harness
