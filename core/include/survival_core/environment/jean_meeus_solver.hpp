#pragma once

#include <cstdint>
#include <cmath>
#include <optional>
#include "survival_core/types.hpp"

namespace survival::environment {

struct SolarPosition {
    double azimuth_rad{0.0};       // Góc phương vị Mặt Trời tính từ Bắc theo chiều kim đồng hồ [rad] [0, 2pi)
    double elevation_rad{0.0};     // Góc nâng Mặt Trời trên đường chân trời [rad] [-pi/2, pi/2]
    double declination_rad{0.0};   // Xích vĩ Mặt Trời (Declination) [rad]
    double hour_angle_rad{0.0};    // Góc giờ địa phương (Local Hour Angle) [rad]
    double equation_of_time_min{0.0}; // Phương trình thời gian [phút]
    bool is_daylight{false};       // Ban ngày (elevation > -0.833 độ = -0.0145 rad có tính khúc xạ khí quyển)
};

/**
 * @brief Bộ giải thiên văn học Mặt Trời chuẩn xác cao Jean Meeus (Astronomical Algorithms)
 * Tính toán vị trí góc Mặt Trời biểu kiến tại bất kỳ tọa độ địa lý (Lat, Lon) và mốc thời gian UTC nào.
 */
class JeanMeeusSolver {
public:
    /**
     * @brief Tính toán vị trí Mặt Trời từ nhãn thời gian Unix nano-giây
     */
    [[nodiscard]] static SolarPosition computeSolarPosition(uint64_t timestamp_ns, double lat_rad, double lon_rad) noexcept;

    /**
     * @brief Tính Ngày Julian (Julian Day) từ năm, tháng, ngày, giờ thập phân UTC
     */
    [[nodiscard]] static double computeJulianDay(int year, int month, int day, double hour_utc) noexcept;

    /**
     * @brief Tính toán hàm Hợp lý (Likelihood) khi có quan sát góc phương vị Mặt Trời / Bóng đổ
     * @param observed_azimuth_rad Góc phương vị quan sát được (từ camera / bóng đổ gậy) [rad]
     * @param expected_azimuth_rad Góc phương vị lý thuyết tính từ Jean Meeus [rad]
     * @param sigma_rad Độ lệch chuẩn sai số đo góc [rad]
     */
    [[nodiscard]] static double computeSolarAzimuthLikelihood(double observed_azimuth_rad, double expected_azimuth_rad, double sigma_rad = 0.05) noexcept;
};

} // namespace survival::environment
