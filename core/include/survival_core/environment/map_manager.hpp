#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <optional>
#include <string>
#include "survival_core/types.hpp"
#include "survival_core/environment/dem_map.hpp"

namespace survival::environment {

struct GpsSample {
    double lat_rad{0.0};
    double lon_rad{0.0};
    double alt_m{0.0};
    double std_pos_m{3.0};
    double hdop{1.0};
    uint64_t timestamp_ns{0};
    bool is_valid{true};
};

struct CleanAnchorResult {
    GpsSample clean_sample{};
    uint64_t delta_time_from_cutoff_ns{0};
    bool is_clean_found{false};
};

/**
 * @brief Bộ đệm trượt 10 mẫu GPS gần nhất để lọc dội sóng (Multipath Poisoning Gating)
 */
class GpsRollingWindow {
public:
    static constexpr size_t kCapacity = 10;
    static constexpr double kMaxAcceptableStdM = 12.0;
    static constexpr double kMaxAcceptableHdop = 2.0;
    static constexpr double kMultipathThresholdStdM = 15.0;

    GpsRollingWindow() noexcept = default;

    void pushSample(const GpsSample& sample) noexcept;

    /**
     * @brief Tìm điểm neo sạch cuối cùng trong quá khứ khi xảy ra mất GPS
     */
    [[nodiscard]] CleanAnchorResult getCleanAnchorAtCutoff(uint64_t cutoff_timestamp_ns) const noexcept;

    [[nodiscard]] size_t getCount() const noexcept {
        return count_;
    }

    void clear() noexcept {
        head_ = 0;
        count_ = 0;
    }

private:
    std::array<GpsSample, kCapacity> buffer_{};
    size_t head_{0};
    size_t count_{0};
};

enum class MapTrackingMode : uint8_t {
    kOnlineGps = 0,             // Có GPS chuẩn xác, liên tục hiệu chuẩn cảm biến
    kTransitionToOffgrid = 1,   // Vừa mất GPS, thực hiện Anchor Lock & Hot-swap DEM
    kOffgridActive = 2          // Hoàn toàn Off-grid: PDR + TRN Particle Filter + Sun
};

struct HandoffContext {
    EastingNorthingAlt anchor_pos{};
    double anchor_std_m{5.0};
    double baro_bias_m{0.0};
    uint64_t time_to_project_pdr_ns{0};
    bool is_valid{false};
};

/**
 * @brief Bộ quản lý 2 luồng bản đồ và điều phối chuyển giao GPS sang Off-Grid mượt mà
 */
class MapManager {
public:
    explicit MapManager(const LocalFrame& frame = LocalFrame{}) noexcept
        : local_frame_(frame)
    {}

    void setLocalFrame(const LocalFrame& frame) noexcept {
        local_frame_ = frame;
    }

    [[nodiscard]] const LocalFrame& getLocalFrame() const noexcept {
        return local_frame_;
    }

    /**
     * @brief Xử lý một mẫu định vị GPS mới có kiểm tra Temporal Consistency
     */
    void handleGpsFix(const GpsSample& fix, double current_raw_baro_alt_m) noexcept;

    /**
     * @brief Kích hoạt chuyển giao khi mất tín hiệu GPS mà không bị nhảy tọa độ về 0,0
     */
    [[nodiscard]] HandoffContext handleGpsCutoff(uint64_t cutoff_timestamp_ns) noexcept;

    /**
     * @brief Lấy bản ghi LastKnownGoodFix mới nhất
     */
    [[nodiscard]] const LastKnownGoodFix& getLastKnownGoodFix() const noexcept {
        return last_good_fix_;
    }

    /**
     * @brief Tính log-likelihood đo lường khi bắt lại được GPS (Soft Bayesian Re-acquisition)
     */
    [[nodiscard]] static double computeSoftGpsLogLikelihood(
        double particle_easting_m,
        double particle_northing_m,
        double gps_easting_m,
        double gps_northing_m,
        double gps_std_m) noexcept;

    [[nodiscard]] MapTrackingMode getTrackingMode() const noexcept {
        return tracking_mode_;
    }

    [[nodiscard]] double getCalibratedBaroBias() const noexcept {
        return baro_bias_m_;
    }

    [[nodiscard]] const GpsRollingWindow& getRollingWindow() const noexcept {
        return rolling_window_;
    }

private:
    MapTrackingMode tracking_mode_{MapTrackingMode::kOnlineGps};
    GpsRollingWindow rolling_window_{};
    LocalFrame local_frame_{};
    LastKnownGoodFix last_good_fix_{};
    double baro_bias_m_{0.0};
    GpsSample last_valid_gps_{};
    bool has_valid_gps_{false};
};

} // namespace survival::environment
