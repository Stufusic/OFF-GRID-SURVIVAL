#include <gtest/gtest.h>
#include "survival_harness/benchmark_runner.hpp"

using namespace survival;
using namespace survival::harness;

TEST(FactorialBenchmarkTest, MonteCarloM0NominalFullFusion) {
    BenchmarkConfig config{};
    config.mode = AlgorithmMode::kFullFusion;
    config.scenario = ScenarioType::kM0_Nominal;
    config.num_monte_carlo_runs = 5;
    config.dataset_step_count = 20;

    BenchmarkRunner runner(config);
    const auto report = runner.run();

    EXPECT_EQ(report.total_runs, 5U);
    EXPECT_GT(report.mean_ce50_m, 0.0);
    EXPECT_LT(report.mean_ce50_m, 35.0); // CE50 < 35m trong điều kiện chuẩn
    EXPECT_LT(report.mean_ce95_m, 80.0); // CE95 < 80m
    EXPECT_TRUE(report.passes_latency_criteria);
}

TEST(FactorialBenchmarkTest, MonteCarloM1MagneticSpikeRejection) {
    BenchmarkConfig config{};
    config.mode = AlgorithmMode::kFullFusion;
    config.scenario = ScenarioType::kM1_MagneticSpike;
    config.num_monte_carlo_runs = 3;
    config.dataset_step_count = 20;

    BenchmarkRunner runner(config);
    const auto report = runner.run();

    // NIS phải phản ánh dị thường từ trường
    EXPECT_GT(report.max_nis, 0.0);
    // Tiêu chuẩn nghiệm thu: CE95 <= 100m dù bị tiêm xung +45 uT
    EXPECT_TRUE(report.passes_m1_magnetic_criteria);
}

TEST(FactorialBenchmarkTest, MonteCarloM2BarometricDriftResilience) {
    BenchmarkConfig config{};
    config.mode = AlgorithmMode::kFullFusion;
    config.scenario = ScenarioType::kM2_BarometricDrift;
    config.num_monte_carlo_runs = 3;
    config.dataset_step_count = 20;

    BenchmarkRunner runner(config);
    const auto report = runner.run();

    // Tiêu chuẩn nghiệm thu: CE95 <= 120m khi áp suất bị trôi 2 hPa/giờ
    EXPECT_TRUE(report.passes_m2_baro_drift_criteria);
}
