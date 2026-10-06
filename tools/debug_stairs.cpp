#include <iostream>
#include <iomanip>
#include <vector>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_harness/csv_loader.hpp"

using namespace survival;
using namespace survival::estimators;
using namespace survival::harness;

int main() {
    auto dataset_opt = CsvLoader::loadSensorLoggerFolder("C:/Users/KieuKhang/Downloads/2026-09-01_10-57-01_1");
    if (!dataset_opt.has_value()) return 1;

    const auto& frames = dataset_opt->sensor_frames;
    MultiModalFusionPipeline pipeline{};
    environment::DemMap empty_dem{};
    pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 100.0}, empty_dem);

    const uint64_t t0 = frames.front().timestamp_ns;

    std::cout << std::left << std::setw(10) << "Time (s)"
              << std::setw(12) << "Cadence(Hz)"
              << std::setw(14) << "Impact(P2P)"
              << std::setw(14) << "StairScore"
              << std::setw(14) << "VehicleScore"
              << std::setw(16) << "Activity"
              << std::setw(12) << "Confidence"
              << "\n";
    std::cout << std::string(85, '-') << "\n";

    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        const double t_s = static_cast<double>(frame.timestamp_ns - t0) * 1e-9;

        auto state_opt = pipeline.processSensorFrame(frame);

        if (state_opt.has_value() && t_s >= 40.0 && t_s <= 52.0) {
            const auto& st = *state_opt;
            const auto& est = pipeline.getActivityClassifier().getLastEstimate();

            std::cout << std::fixed << std::setprecision(2)
                      << std::left << std::setw(10) << t_s
                      << std::setw(12) << est.evidence.cadence_hz
                      << std::setw(14) << est.evidence.impact_score
                      << std::setw(14) << est.evidence.stair_score
                      << std::setw(14) << est.evidence.vehicle_score
                      << std::setw(16) << static_cast<int>(est.current_activity)
                      << std::setw(12) << est.fused_confidence
                      << "\n";
        }
    }
    return 0;
}
