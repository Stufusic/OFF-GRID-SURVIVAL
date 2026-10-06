#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_harness/failure_injector.hpp"
#include "survival_harness/synthetic_generator.hpp"
#include "survival_harness/metrics_collector.hpp"
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/dem_map.hpp"

namespace survival::harness {

enum class AlgorithmMode : uint8_t {
    kPurePDR = 0,              // Chỉ tích phân PDR quán tính thô
    kEskfPDR = 1,              // PDR + ESKF 3D Attitude Filter
    kDemParticleFilter = 2,    // ESKF + PDR + Bản đồ địa hình DEM TRN
    kFullFusion = 3            // Full Fusion: ESKF + PDR + DEM + Jean Meeus Sun Ephemeris
};

struct BenchmarkConfig {
    AlgorithmMode mode{AlgorithmMode::kFullFusion};
    ScenarioType scenario{ScenarioType::kM0_Nominal};
    size_t num_monte_carlo_runs{10};
    size_t dataset_step_count{30};
    
    // Khóa hạt giống ngẫu nhiên kép (Dual-Seed Determinism)
    uint32_t environment_seed{1337}; // Seed 1: Cố định kịch bản nhiễu môi trường
    uint32_t estimator_seed{42};     // Seed 2: Điều khiển số ngẫu nhiên bộ lọc hạt

    // DEM địa hình
    double dem_width_m{3000.0};
    double dem_height_m{3000.0};
    double dem_peak_alt_m{2000.0};
    double dem_base_alt_m{1000.0};
};

struct BenchmarkReport {
    AlgorithmMode mode{AlgorithmMode::kFullFusion};
    ScenarioType scenario{ScenarioType::kM0_Nominal};
    size_t total_runs{0};

    double mean_ce50_m{0.0};
    double mean_ce95_m{0.0};
    double mean_nees{0.0};
    double max_nis{0.0};
    double p_loss{0.0};
    
    double avg_fast_loop_cpu_us{0.0};
    double avg_particle_step_cpu_ms{0.0};
    bool passes_m1_magnetic_criteria{false}; // CE95 <= 100m dưới +45 uT spike
    bool passes_m2_baro_drift_criteria{false}; // CE95 tăng <= 20% so với M0 (chạy baseline nội bộ)
    double m0_baseline_ce95_m{0.0};            // [C4] CE95 của lượt M0 ghép cặp cùng seed
    bool passes_latency_criteria{false};      // Fast Loop <= 50 us, PF <= 2.0 ms
};

/**
 * @brief Đấu trường Thực nghiệm Monte Carlo Đa Yếu tố (Benchmark Runner)
 */
class BenchmarkRunner {
public:
    explicit BenchmarkRunner(const BenchmarkConfig& config = BenchmarkConfig{}) noexcept;

    /**
     * @brief Chạy ma trận thực nghiệm Monte Carlo
     */
    [[nodiscard]] BenchmarkReport run() noexcept;

    void setConfig(const BenchmarkConfig& config) noexcept {
        config_ = config;
    }

private:
    BenchmarkConfig config_{};
};

} // namespace survival::harness
