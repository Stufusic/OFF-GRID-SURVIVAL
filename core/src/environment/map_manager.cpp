#include "survival_core/environment/map_manager.hpp"
#include <cmath>
#include <algorithm>

namespace survival::environment {

void GpsRollingWindow::pushSample(const GpsSample& sample) noexcept {
    buffer_[head_] = sample;
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity) {
        count_++;
    }
}

CleanAnchorResult GpsRollingWindow::getCleanAnchorAtCutoff(uint64_t cutoff_timestamp_ns) const noexcept {
    CleanAnchorResult result{};
    if (count_ == 0) {
        return result;
    }

    // Duyệt ngược từ mẫu mới nhất về quá khứ trong bộ đệm trượt
    for (size_t i = 0; i < count_; ++i) {
        const size_t idx = (head_ + kCapacity - 1 - i) % kCapacity;
        const auto& candidate = buffer_[idx];

        if (candidate.is_valid &&
            candidate.std_pos_m <= kMaxAcceptableStdM &&
            candidate.hdop < kMaxAcceptableHdop) {
            
            result.clean_sample = candidate;
            result.is_clean_found = true;
            if (cutoff_timestamp_ns >= candidate.timestamp_ns) {
                result.delta_time_from_cutoff_ns = cutoff_timestamp_ns - candidate.timestamp_ns;
            } else {
                result.delta_time_from_cutoff_ns = 0;
            }
            return result;
        }
    }

    // Nếu không có mẫu nào thỏa mãn gating chặt, lấy mẫu có sai số nhỏ nhất
    double min_std = 1e9;
    size_t best_idx = 0;
    for (size_t i = 0; i < count_; ++i) {
        const size_t idx = (head_ + kCapacity - 1 - i) % kCapacity;
        if (buffer_[idx].is_valid && buffer_[idx].std_pos_m < min_std) {
            min_std = buffer_[idx].std_pos_m;
            best_idx = idx;
        }
    }

    result.clean_sample = buffer_[best_idx];
    result.is_clean_found = true;
    if (cutoff_timestamp_ns >= buffer_[best_idx].timestamp_ns) {
        result.delta_time_from_cutoff_ns = cutoff_timestamp_ns - buffer_[best_idx].timestamp_ns;
    }
    return result;
}

void MapManager::handleGpsFix(const GpsSample& fix, double current_raw_baro_alt_m) noexcept {
    if (!fix.is_valid) {
        return;
    }

    // Temporal Consistency Gate: kiểm tra vận tốc nhảy tức thời
    if (has_valid_gps_ && fix.timestamp_ns > last_valid_gps_.timestamp_ns) {
        const double dt_s = static_cast<double>(fix.timestamp_ns - last_valid_gps_.timestamp_ns) * 1e-9;
        if (dt_s > 1e-3) {
            const double d_lat_m = (fix.lat_rad - last_valid_gps_.lat_rad) * kEarthRadiusMeanM;
            const double cos_lat = std::cos(last_valid_gps_.lat_rad);
            const double d_lon_m = (fix.lon_rad - last_valid_gps_.lon_rad) * (kEarthRadiusMeanM * cos_lat);
            const double dist_m = std::sqrt(d_lat_m * d_lat_m + d_lon_m * d_lon_m);
            const double v_gps = dist_m / dt_s;

            // Nếu vận tốc nhảy quá lớn (> 30 m/s ~ 108 km/h khi đang đi bộ) -> Gating loại bỏ
            if (v_gps > 30.0 && fix.std_pos_m > 10.0) {
                return;
            }
        }
    }

    rolling_window_.pushSample(fix);

    if (fix.std_pos_m <= GpsRollingWindow::kMaxAcceptableStdM && fix.hdop < GpsRollingWindow::kMaxAcceptableHdop) {
        // Online Auto-Calibration: Cập nhật độ trôi khí áp kế
        const double instant_bias = fix.alt_m - current_raw_baro_alt_m;
        if (!has_valid_gps_) {
            baro_bias_m_ = instant_bias;
            // Khởi tạo origin cho LocalFrame nếu chưa được set
            if (std::abs(local_frame_.origin_lat_rad) < 1e-6 && std::abs(local_frame_.origin_lon_rad) < 1e-6) {
                local_frame_.origin_lat_rad = fix.lat_rad;
                local_frame_.origin_lon_rad = fix.lon_rad;
                local_frame_.origin_alt_m = fix.alt_m;
            }
        } else {
            baro_bias_m_ = 0.95 * baro_bias_m_ + 0.05 * instant_bias;
        }

        last_valid_gps_ = fix;
        has_valid_gps_ = true;
        tracking_mode_ = MapTrackingMode::kOnlineGps;

        // Cập nhật LastKnownGoodFix với cả WGS-84 và ENU
        GeodeticCoord geo{
            .latitude_deg = fix.lat_rad * kRadToDeg,
            .longitude_deg = fix.lon_rad * kRadToDeg,
            .altitude_m = fix.alt_m
        };
        const auto enu = local_frame_.geodeticToEnu(geo);

        last_good_fix_.latitude_deg = geo.latitude_deg;
        last_good_fix_.longitude_deg = geo.longitude_deg;
        last_good_fix_.altitude_m = geo.altitude_m;
        last_good_fix_.easting_m = enu.easting_m;
        last_good_fix_.northing_m = enu.northing_m;
        last_good_fix_.timestamp_ns = fix.timestamp_ns;
        last_good_fix_.horizontal_accuracy_m = static_cast<float>(fix.std_pos_m);
        last_good_fix_.vertical_accuracy_m = static_cast<float>(fix.std_pos_m * 1.5);
        last_good_fix_.is_valid = true;
    }
}

HandoffContext MapManager::handleGpsCutoff(uint64_t cutoff_timestamp_ns) noexcept {
    HandoffContext ctx{};
    const auto anchor_res = rolling_window_.getCleanAnchorAtCutoff(cutoff_timestamp_ns);

    if (anchor_res.is_clean_found) {
        GeodeticCoord geo{
            .latitude_deg = anchor_res.clean_sample.lat_rad * kRadToDeg,
            .longitude_deg = anchor_res.clean_sample.lon_rad * kRadToDeg,
            .altitude_m = anchor_res.clean_sample.alt_m
        };
        const auto enu = local_frame_.geodeticToEnu(geo);

        ctx.anchor_pos.easting_m = enu.easting_m;
        ctx.anchor_pos.northing_m = enu.northing_m;
        ctx.anchor_pos.altitude_m = anchor_res.clean_sample.alt_m;
        ctx.anchor_std_m = std::max(2.0, anchor_res.clean_sample.std_pos_m);
        ctx.baro_bias_m = baro_bias_m_;
        ctx.time_to_project_pdr_ns = anchor_res.delta_time_from_cutoff_ns;
        ctx.is_valid = true;
    } else if (has_valid_gps_) {
        GeodeticCoord geo{
            .latitude_deg = last_valid_gps_.lat_rad * kRadToDeg,
            .longitude_deg = last_valid_gps_.lon_rad * kRadToDeg,
            .altitude_m = last_valid_gps_.alt_m
        };
        const auto enu = local_frame_.geodeticToEnu(geo);

        ctx.anchor_pos.easting_m = enu.easting_m;
        ctx.anchor_pos.northing_m = enu.northing_m;
        ctx.anchor_pos.altitude_m = last_valid_gps_.alt_m;
        ctx.anchor_std_m = last_valid_gps_.std_pos_m;
        ctx.baro_bias_m = baro_bias_m_;
        ctx.time_to_project_pdr_ns = (cutoff_timestamp_ns > last_valid_gps_.timestamp_ns) ? (cutoff_timestamp_ns - last_valid_gps_.timestamp_ns) : 0;
        ctx.is_valid = true;
    }

    tracking_mode_ = MapTrackingMode::kOffgridActive;
    return ctx;
}

double MapManager::computeSoftGpsLogLikelihood(
    double particle_easting_m,
    double particle_northing_m,
    double gps_easting_m,
    double gps_northing_m,
    double gps_std_m) noexcept {
    
    const double de = particle_easting_m - gps_easting_m;
    const double dn = particle_northing_m - gps_northing_m;
    const double dist_sq = de * de + dn * dn;

    const double var = std::max(1.0, gps_std_m * gps_std_m);
    return -dist_sq / (2.0 * var);
}

} // namespace survival::environment
