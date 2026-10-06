#include "survival_core/environment/jean_meeus_solver.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>
#include <algorithm>

namespace survival::environment {

double JeanMeeusSolver::computeJulianDay(int year, int month, int day, double hour_utc) noexcept {
    int y = year;
    int m = month;
    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    const int a = y / 100;
    const int b = 2 - a + (a / 4);

    const double jd = std::floor(365.25 * (y + 4716)) +
                      std::floor(30.6001 * (m + 1)) +
                      static_cast<double>(day) +
                      (hour_utc / 24.0) +
                      static_cast<double>(b) - 1524.5;
    return jd;
}

SolarPosition JeanMeeusSolver::computeSolarPosition(uint64_t timestamp_ns, double lat_rad, double lon_rad) noexcept {
    // 1. Chuyển Unix timestamp sang Ngày Julian JD
    const double unix_seconds = static_cast<double>(timestamp_ns) * 1e-9;
    // Unix epoch 1970-01-01 00:00:00 UTC = JD 2440587.5
    const double jd = 2440587.5 + (unix_seconds / 86400.0);

    // 2. Thế kỷ Julian J2000.0
    const double T = (jd - 2451545.0) / 36525.0;

    // 3. Kinh độ trung bình Mặt Trời L0 (độ)
    double L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;
    L0 = std::fmod(L0, 360.0);
    if (L0 < 0.0) L0 += 360.0;

    // 4. Dị thường trung bình Mặt Trời M (độ)
    double M_deg = 357.52911 + 35999.05029 * T - 0.0001537 * T * T;
    M_deg = std::fmod(M_deg, 360.0);
    if (M_deg < 0.0) M_deg += 360.0;
    const double M_rad = math::degToRad(M_deg);

    // 5. Độ lệch tâm quỹ đạo Trái Đất e
    const double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;

    // 6. Phương trình tâm Sun Equation of Center C (độ)
    const double C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * std::sin(M_rad) +
                     (0.019993 - 0.000101 * T) * std::sin(2.0 * M_rad) +
                     0.000289 * std::sin(3.0 * M_rad);

    // 7. Kinh độ thực tế Sun True Longitude (độ)
    const double sun_true_lon_deg = L0 + C;
    const double sun_true_lon_rad = math::degToRad(sun_true_lon_deg);

    // 8. Độ nghiêng hoàng đạo trung bình Mean Obliquity eps0 (độ)
    const double eps0 = 23.439291 - 0.0130042 * T - 0.00000016 * T * T + 0.000000504 * T * T * T;

    // 9. Bù trừ Nutation (Góc nút Mặt trăng Omega)
    const double omega_deg = 125.04 - 1934.136 * T;
    const double omega_rad = math::degToRad(omega_deg);
    const double lambda_deg = sun_true_lon_deg - 0.00569 - 0.00478 * std::sin(omega_rad);
    const double lambda_rad = math::degToRad(lambda_deg);
    const double eps_rad = math::degToRad(eps0 + 0.00256 * std::cos(omega_rad));

    // 10. Xích vĩ Mặt Trời (Declination delta)
    const double sin_delta = std::sin(eps_rad) * std::sin(lambda_rad);
    const double delta_rad = std::asin(std::clamp(sin_delta, -1.0, 1.0));

    // 11. Xích kinh Mặt Trời (Right Ascension alpha)
    const double alpha_rad = std::atan2(std::cos(eps_rad) * std::sin(lambda_rad), std::cos(lambda_rad));

    // 12. Giờ thiên văn Greenwich (Greenwich Mean Sidereal Time - GMST)
    double gmst_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0) +
                      0.000387933 * T * T - (T * T * T / 38710000.0);
    gmst_deg = std::fmod(gmst_deg, 360.0);
    if (gmst_deg < 0.0) gmst_deg += 360.0;
    const double gmst_rad = math::degToRad(gmst_deg);

    // 13. Góc giờ địa phương (Local Hour Angle H)
    const double H_rad = math::normalizeAngleRad(gmst_rad + lon_rad - alpha_rad);

    // 14. Góc nâng Mặt Trời (Solar Elevation gamma)
    const double sin_elevation = std::sin(lat_rad) * std::sin(delta_rad) +
                                 std::cos(lat_rad) * std::cos(delta_rad) * std::cos(H_rad);
    const double elevation_rad = std::asin(std::clamp(sin_elevation, -1.0, 1.0));

    // 15. Góc phương vị Mặt Trời (Solar Azimuth psi_sun) từ Bắc theo chiều kim đồng hồ
    // tan(Az) = -sin(H) / (cos(Lat)*tan(delta) - sin(Lat)*cos(H))
    const double y_az = -std::sin(H_rad) * std::cos(delta_rad);
    const double x_az = std::cos(lat_rad) * std::sin(delta_rad) -
                        std::sin(lat_rad) * std::cos(delta_rad) * std::cos(H_rad);
    const double azimuth_rad = math::normalizeHeadingRad(std::atan2(y_az, x_az));

    // 16. Phương trình thời gian Equation of Time (phút)
    const double y_param = std::tan(eps_rad / 2.0) * std::tan(eps_rad / 2.0);
    const double eot_rad = y_param * std::sin(2.0 * math::degToRad(L0)) -
                           2.0 * e * std::sin(M_rad) +
                           4.0 * e * y_param * std::sin(M_rad) * std::cos(2.0 * math::degToRad(L0)) -
                           0.5 * y_param * y_param * std::sin(4.0 * math::degToRad(L0)) -
                           1.25 * e * e * std::sin(2.0 * M_rad);

    SolarPosition pos{};
    pos.azimuth_rad = azimuth_rad;
    pos.elevation_rad = elevation_rad;
    pos.declination_rad = delta_rad;
    pos.hour_angle_rad = H_rad;
    pos.equation_of_time_min = math::radToDeg(eot_rad) * 4.0; // 1 độ = 4 phút
    pos.is_daylight = (elevation_rad > -0.0145); // -0.833 độ có tính khúc xạ

    return pos;
}

double JeanMeeusSolver::computeSolarAzimuthLikelihood(
    double observed_azimuth_rad, double expected_azimuth_rad, double sigma_rad) noexcept {
    const double diff_rad = math::angleDifferenceRad(observed_azimuth_rad, expected_azimuth_rad);
    const double var = sigma_rad * sigma_rad;
    if (var < 1e-12) return 1.0;
    return std::exp(-0.5 * (diff_rad * diff_rad) / var);
}

} // namespace survival::environment
