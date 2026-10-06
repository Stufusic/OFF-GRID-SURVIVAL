#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include "survival_core/types.hpp"

namespace survival::harness {

struct NavigationMetricReport {
    size_t sample_count{0};
    
    // Các chỉ số sai số vị trí mặt phẳng ngang [m]
    double ce50_m{0.0};                     // Sai số vòng tròn xác suất 50% (Circular Error Probable)
    double ce95_m{0.0};                     // Sai số vòng tròn xác suất 95% (95th Percentile Error)
    double rmse_2d_m{0.0};                  // Sai số toàn phương trung bình 2D
    double rmse_3d_m{0.0};                  // Sai số toàn phương trung bình 3D
    double mae_2d_m{0.0};                   // Sai số tuyệt đối trung bình 2D
    double max_error_2d_m{0.0};             // Sai số lớn nhất 2D

    // Các chỉ số tính nhất quán thống kê (Statistical Consistency)
    double mean_nees_2d{0.0};               // NEES trung bình 2D (Kỳ vọng = 2.0)
    double nees_in_bounds_percent{0.0};     // Tỷ lệ % mẫu NEES nằm trong ngưỡng 95% Chi-square
    double empirical_containment_95{0.0};   // Tỷ lệ % thực tế vị trí thật nằm trong elip CE95 ước lượng
    
    // Sai số góc phương vị [deg]
    double mean_heading_error_deg{0.0};
    double max_heading_error_deg{0.0};

    // Chỉ số NIS đo lường cập nhật
    double mean_nis{0.0};

    [[nodiscard]] std::string formatSummary() const;
};

struct SampleRecord {
    uint64_t timestamp_ns{0};
    EastingNorthingAlt estimated_pos{};
    EastingNorthingAlt ground_truth_pos{};
    std::array<double, 4> cov_2d_flat{1.0, 0.0, 0.0, 1.0}; // [var_e, cov_en, cov_en, var_n]
    double estimated_heading_rad{0.0};
    double ground_truth_heading_rad{0.0};
    double innovation{0.0};
    double innovation_var{1.0};
    bool has_innovation{false};
};

class MetricsCollector {
public:
    MetricsCollector() = default;

    /**
     * @brief Ghi nhận 1 mẫu ước lượng kèm Ground Truth và Hiệp phương sai
     */
    void recordEstimate(
        uint64_t timestamp_ns,
        const EastingNorthingAlt& estimated_pos,
        const EastingNorthingAlt& ground_truth_pos,
        double var_e,
        double var_n,
        double cov_en = 0.0,
        double estimated_heading_rad = 0.0,
        double ground_truth_heading_rad = 0.0) noexcept;

    /**
     * @brief Ghi nhận mẫu Innovation phục vụ tính NIS
     */
    void recordInnovation(
        uint64_t timestamp_ns,
        double innovation,
        double innovation_variance) noexcept;

    /**
     * @brief Tính toán toàn bộ báo cáo chỉ số kiểm định
     */
    [[nodiscard]] NavigationMetricReport computeReport() const noexcept;

    /**
     * @brief Xóa toàn bộ dữ liệu mẫu đã ghi
     */
    void reset() noexcept {
        records_.clear();
    }

    [[nodiscard]] size_t getSampleCount() const noexcept {
        return records_.size();
    }

private:
    std::vector<SampleRecord> records_{};
};

} // namespace survival::harness
