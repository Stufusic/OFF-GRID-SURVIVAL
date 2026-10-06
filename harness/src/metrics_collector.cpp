#include "survival_harness/metrics_collector.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_core/math/statistics.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace survival::harness {

void MetricsCollector::recordEstimate(
    uint64_t timestamp_ns,
    const EastingNorthingAlt& estimated_pos,
    const EastingNorthingAlt& ground_truth_pos,
    double var_e,
    double var_n,
    double cov_en,
    double estimated_heading_rad,
    double ground_truth_heading_rad) noexcept
{
    SampleRecord record{};
    record.timestamp_ns = timestamp_ns;
    record.estimated_pos = estimated_pos;
    record.ground_truth_pos = ground_truth_pos;
    record.cov_2d_flat = {var_e, cov_en, cov_en, var_n};
    record.estimated_heading_rad = estimated_heading_rad;
    record.ground_truth_heading_rad = ground_truth_heading_rad;
    record.has_innovation = false;

    records_.push_back(record);
}

void MetricsCollector::recordInnovation(
    uint64_t timestamp_ns,
    double innovation,
    double innovation_variance) noexcept
{
    SampleRecord record{};
    record.timestamp_ns = timestamp_ns;
    record.innovation = innovation;
    record.innovation_var = std::max(innovation_variance, 1e-9);
    record.has_innovation = true;

    records_.push_back(record);
}

NavigationMetricReport MetricsCollector::computeReport() const noexcept {
    NavigationMetricReport report{};
    if (records_.empty()) {
        return report;
    }

    std::vector<double> errors_2d;
    errors_2d.reserve(records_.size());

    double sum_sq_2d = 0.0;
    double sum_sq_3d = 0.0;
    double sum_abs_2d = 0.0;
    double max_err_2d = 0.0;

    double sum_nees = 0.0;
    size_t nees_in_bounds_count = 0;
    size_t containment_95_count = 0;
    size_t position_sample_count = 0;

    double sum_heading_err = 0.0;
    double max_heading_err = 0.0;

    double sum_nis = 0.0;
    size_t nis_count = 0;

    for (const auto& r : records_) {
        if (r.has_innovation) {
            const double nis = (r.innovation * r.innovation) / r.innovation_var;
            sum_nis += nis;
            nis_count++;
            continue;
        }

        position_sample_count++;

        const double de = r.estimated_pos.easting_m - r.ground_truth_pos.easting_m;
        const double dn = r.estimated_pos.northing_m - r.ground_truth_pos.northing_m;
        const double dh = r.estimated_pos.altitude_m - r.ground_truth_pos.altitude_m;

        const double err_2d = std::sqrt(de * de + dn * dn);
        const double err_3d = std::sqrt(de * de + dn * dn + dh * dh);

        errors_2d.push_back(err_2d);
        sum_sq_2d += err_2d * err_2d;
        sum_sq_3d += err_3d * err_3d;
        sum_abs_2d += err_2d;
        max_err_2d = std::max(max_err_2d, err_2d);

        // Tính NEES 2D: e^T * P^-1 * e
        const double v_e = std::max(r.cov_2d_flat[0], 1e-8);
        const double cov_en = r.cov_2d_flat[1];
        const double v_n = std::max(r.cov_2d_flat[3], 1e-8);

        double det = v_e * v_n - cov_en * cov_en;
        double nees = 0.0;
        if (det > 1e-12) {
            nees = (v_n * de * de - 2.0 * cov_en * de * dn + v_e * dn * dn) / det;
        } else {
            nees = (de * de / v_e) + (dn * dn / v_n);
        }

        sum_nees += nees;
        // Ngưỡng Chi-Square 2 DOF (95% CI: [0.0513, 7.3778])
        if (nees >= 0.0513 && nees <= 7.3778) {
            nees_in_bounds_count++;
        }

        // Kiểm tra Containment 95%: Vị trí thật nằm trong elip 95% khi NEES <= 5.9915
        if (nees <= math::kChiSquare2D_95) {
            containment_95_count++;
        }

        // Sai số góc phương vị
        const double heading_diff_deg = std::abs(math::radToDeg(math::angleDifferenceRad(
            r.estimated_heading_rad, r.ground_truth_heading_rad
        )));
        sum_heading_err += heading_diff_deg;
        max_heading_err = std::max(max_heading_err, heading_diff_deg);
    }

    report.sample_count = position_sample_count;

    if (position_sample_count > 0) {
        std::sort(errors_2d.begin(), errors_2d.end());
        const size_t idx_50 = static_cast<size_t>(0.50 * static_cast<double>(position_sample_count - 1));
        const size_t idx_95 = static_cast<size_t>(0.95 * static_cast<double>(position_sample_count - 1));

        report.ce50_m = errors_2d[idx_50];
        report.ce95_m = errors_2d[idx_95];
        report.rmse_2d_m = std::sqrt(sum_sq_2d / static_cast<double>(position_sample_count));
        report.rmse_3d_m = std::sqrt(sum_sq_3d / static_cast<double>(position_sample_count));
        report.mae_2d_m = sum_abs_2d / static_cast<double>(position_sample_count);
        report.max_error_2d_m = max_err_2d;

        report.mean_nees_2d = sum_nees / static_cast<double>(position_sample_count);
        report.nees_in_bounds_percent = (static_cast<double>(nees_in_bounds_count) / static_cast<double>(position_sample_count)) * 100.0;
        report.empirical_containment_95 = (static_cast<double>(containment_95_count) / static_cast<double>(position_sample_count)) * 100.0;

        report.mean_heading_error_deg = sum_heading_err / static_cast<double>(position_sample_count);
        report.max_heading_error_deg = max_heading_err;
    }

    if (nis_count > 0) {
        report.mean_nis = sum_nis / static_cast<double>(nis_count);
    }

    return report;
}

std::string NavigationMetricReport::formatSummary() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "========================================================\n";
    oss << "        OFF-GRID SURVIVAL ENGINE - METRIC REPORT        \n";
    oss << "========================================================\n";
    oss << " Total Position Samples   : " << sample_count << "\n";
    oss << " CE50 (Median Error)      : " << ce50_m << " m\n";
    oss << " CE95 (95% Bound Error)   : " << ce95_m << " m\n";
    oss << " RMSE 2D / 3D             : " << rmse_2d_m << " m / " << rmse_3d_m << " m\n";
    oss << " Mean Absolute Error 2D   : " << mae_2d_m << " m (Max: " << max_error_2d_m << " m)\n";
    oss << "--------------------------------------------------------\n";
    oss << " Mean NEES 2D (Ideal: 2.0): " << mean_nees_2d << "\n";
    oss << " NEES in 95% Bounds       : " << nees_in_bounds_percent << " %\n";
    oss << " Empirical Containment 95%: " << empirical_containment_95 << " %\n";
    oss << " Mean Heading Error       : " << mean_heading_error_deg << " deg (Max: " << max_heading_error_deg << " deg)\n";
    oss << "========================================================\n";
    return oss.str();
}

} // namespace survival::harness
