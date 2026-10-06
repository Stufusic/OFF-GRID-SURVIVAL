// ============================================================================
// [C5] REGRESSION TEST TRÊN DỮ LIỆU THẬT (MotionSense, Sensor Logger format)
// ============================================================================
// Khóa hành vi pipeline trên 16 recordings CỐ ĐỊNH (4 mỗi loại hoạt động có
// nhãn người-gán). Golden values chụp từ build đã qua benchmark phase 1-3
// (2026-09-28). Dải cho phép ±20% — đủ chặn hồi quy lớn, không khoá cứng cả
// tối ưu nhỏ. Nếu thay đổi có chủ đích làm số thay đổi: chạy lại
// scripts/run_benchmarks.py, cập nhật bảng golden và ghi rõ trong PR.
//
// Test tự SKIP khi testdata không tồn tại (máy khác vẫn build/chạy được).

#include <gtest/gtest.h>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_harness/csv_loader.hpp"

#include <filesystem>
#include <string>
#include <vector>

using namespace survival;
using namespace survival::estimators;
using namespace survival::harness;

namespace {

struct GoldenRecording {
    const char* folder;
    int golden_steps;
    const char* true_activity;  // dws=STAIRS_DESCENT, ups=STAIRS_ASCENT,
                                // wlk=WALKING_FLAT, jog=RUNNING, sit=STATIONARY
};

const std::vector<GoldenRecording>& goldenSet() {
    static const std::vector<GoldenRecording> kSet = {
        {"dws_1_sub1", 60, "STAIRS_DESCENT"},
        {"dws_2_sub3", 112, "STAIRS_DESCENT"},
        {"dws_11_sub14", 19, "STAIRS_DESCENT"},
        {"dws_1_sub16", 71, "STAIRS_DESCENT"},
        {"ups_3_sub9", 101, "STAIRS_ASCENT"},
        {"ups_4_sub2", 102, "STAIRS_ASCENT"},
        {"ups_12_sub18", 40, "STAIRS_ASCENT"},
        {"ups_12_sub1", 35, "STAIRS_ASCENT"},
        {"wlk_7_sub1", 203, "WALKING_FLAT"},
        {"wlk_8_sub2", 165, "WALKING_FLAT"},
        {"wlk_15_sub10", 87, "WALKING_FLAT"},
        {"wlk_15_sub16", 104, "WALKING_FLAT"},
        {"jog_9_sub16", 129, "RUNNING"},
        {"jog_16_sub1", 53, "RUNNING"},
        {"jog_16_sub10", 35, "RUNNING"},
        {"sit_5_sub3", 1, "STATIONARY"},  // [A1] 10->1: bước ma 1Hz bị cổng p2p chặn (cải tiến có chủ đích)
    };
    return kSet;
}

std::filesystem::path findTestdataRoot() {
    // 1) Ưu tiên biến môi trường
    if (const char* env = std::getenv("SURVIVAL_TESTDATA")) {
        std::filesystem::path p(env);
        if (std::filesystem::exists(p)) return p;
    }
    // 2) Các đường tương đối phổ biến tùy thư mục chạy ctest (build/ hoặc build/tests/)
    for (const auto& cand : {"../testdata", "../../testdata", "../../../testdata",
                             "../../../../testdata", "../../../../../../testdata"}) {
        std::filesystem::path p(cand);
        if (std::filesystem::exists(p / "motionsense" / "manifest.csv")) return p;
    }
    return {};
}

int runPipelineSteps(const std::filesystem::path& folder) {
    auto dataset = CsvLoader::loadSensorLoggerFolder(folder.string());
    if (!dataset.has_value() || dataset->sensor_frames.empty()) {
        return -1;
    }
    MultiModalFusionPipeline pipeline{};
    environment::DemMap empty_dem{};
    pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 100.0}, empty_dem);
    for (const auto& frame : dataset->sensor_frames) {
        (void)pipeline.processSensorFrame(frame);
    }
    return static_cast<int>(pipeline.getTotalSteps());
}

} // namespace

TEST(RealDataRegression, MotionSenseGoldenStepCounts) {
    const auto root = findTestdataRoot();
    if (root.empty()) {
        GTEST_SKIP() << "testdata/motionsense không có trên máy này (set SURVIVAL_TESTDATA để bật)";
    }

    int checked = 0;
    for (const auto& g : goldenSet()) {
        const auto folder = root / "motionsense" / g.folder;
        ASSERT_TRUE(std::filesystem::exists(folder)) << "Thiếu: " << folder.string();
        const int steps = runPipelineSteps(folder);
        ASSERT_GE(steps, 0) << "Không load được: " << folder.string();

        // Golden ±20%, sàn tuyệt đối 3 bước (tránh pass khi 0 bước)
        const int tol = std::max(3, static_cast<int>(0.20 * g.golden_steps));
        EXPECT_LE(steps, g.golden_steps + tol)
            << "[" << g.folder << "/" << g.true_activity << "] bước tăng bất thường";
        EXPECT_GE(steps, g.golden_steps - tol)
            << "[" << g.folder << "/" << g.true_activity << "] bước giảm bất thường";
        checked++;
    }
    EXPECT_EQ(checked, 16);
}
