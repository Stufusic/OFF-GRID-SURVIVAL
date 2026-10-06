#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <string_view>
#include <optional>

namespace survival {

// ============================================================================
// HẰNG SỐ VẬT LÝ VÀ TOÁN HỌC TIÊU CHUẨN
// ============================================================================
inline constexpr double kGravityStandard = 9.80665;              // [m/s^2] Gia tốc trọng trường chuẩn
// [A1] Chiều rộng mặt bậc thang ngang — TRƯỚC đây có 3 nguồn sự thật lệch nhau
// (0.27 pipeline / 0.28 velocity-estimator ascent / 0.30 descent) — thống nhất 1 hằng số.
inline constexpr double kStairTreadM = 0.28;                     // [m] Chiều rộng mặt bậc thang
inline constexpr double kStandardPressureHpa = 1013.25;          // [hPa] Áp suất khí quyển mực nước biển chuẩn
inline constexpr double kPi = 3.14159265358979323846;           // Số Pi
inline constexpr double kTwoPi = 2.0 * kPi;                     // 2*Pi
inline constexpr double kHalfPi = 0.5 * kPi;                    // Pi/2
inline constexpr double kDegToRad = kPi / 180.0;                // Hệ số đổi Deg -> Rad
inline constexpr double kRadToDeg = 180.0 / kPi;                // Hệ số đổi Rad -> Deg
inline constexpr double kEarthRadiusMeanM = 6371000.0;          // [m] Bán kính trung bình Trái Đất

// ============================================================================
// CẤU TRÚC VECTOR & TOẠ ĐỘ ENU (Zero SIMD-Alignment Trap)
// ============================================================================
struct Vector3f {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};

    [[nodiscard]] constexpr std::array<float, 3> toArray() const noexcept {
        return {x, y, z};
    }
};

struct Vector3d {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] constexpr std::array<double, 3> toArray() const noexcept {
        return {x, y, z};
    }

    [[nodiscard]] double norm() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }
};

// Quaternion thái độ định hướng: q = [w, x, y, z] (w là phần thực)
struct Quaterniond {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] double norm() const noexcept {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    void normalize() noexcept {
        const double n = norm();
        if (n > 1e-12) {
            const double inv_n = 1.0 / n;
            w *= inv_n;
            x *= inv_n;
            y *= inv_n;
            z *= inv_n;
        } else {
            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }
    }
};

// Tọa độ phẳng cục bộ ENU (East-North-Up)
struct EastingNorthingAlt {
    double easting_m{0.0};    // [m] Trục Đông
    double northing_m{0.0};   // [m] Trục Bắc
    double altitude_m{0.0};   // [m] Cao độ
};

// Tọa độ địa lý WGS-84
struct GeodeticCoord {
    double latitude_deg{0.0};   // [deg] Vĩ độ [-90, 90]
    double longitude_deg{0.0};  // [deg] Kinh độ [-180, 180]
    double altitude_m{0.0};     // [m] Cao độ so với mực nước biển
};

// ============================================================================
// HỆ QUY CHIẾU CỤC BỘ DÙNG CHUNG (Shared Local Frame)
// ============================================================================
struct LocalFrame {
    double origin_lat_rad{0.0};
    double origin_lon_rad{0.0};
    double origin_alt_m{0.0};

    [[nodiscard]] EastingNorthingAlt geodeticToEnu(const GeodeticCoord& geo) const noexcept {
        const double lat_rad = geo.latitude_deg * kDegToRad;
        const double lon_rad = geo.longitude_deg * kDegToRad;
        const double delta_lat = lat_rad - origin_lat_rad;
        const double delta_lon = lon_rad - origin_lon_rad;
        const double cos_lat = std::max(0.01, std::cos(origin_lat_rad));

        return EastingNorthingAlt{
            .easting_m = delta_lon * (kEarthRadiusMeanM * cos_lat),
            .northing_m = delta_lat * kEarthRadiusMeanM,
            .altitude_m = geo.altitude_m
        };
    }

    [[nodiscard]] GeodeticCoord enuToGeodetic(const EastingNorthingAlt& enu) const noexcept {
        const double delta_lat = enu.northing_m / kEarthRadiusMeanM;
        const double lat_rad = origin_lat_rad + delta_lat;
        const double cos_lat = std::max(0.01, std::cos(origin_lat_rad));
        const double delta_lon = enu.easting_m / (kEarthRadiusMeanM * cos_lat);
        const double lon_rad = origin_lon_rad + delta_lon;

        return GeodeticCoord{
            .latitude_deg = lat_rad * kRadToDeg,
            .longitude_deg = lon_rad * kRadToDeg,
            .altitude_m = enu.altitude_m
        };
    }
};

// ============================================================================
// ĐIỂM FIX GPS HỢP LỆ CUỐI CÙNG (Last Known Good Fix)
// ============================================================================
struct LastKnownGoodFix {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    double easting_m{0.0};
    double northing_m{0.0};
    uint64_t timestamp_ns{0};
    float horizontal_accuracy_m{999.0F};
    float vertical_accuracy_m{999.0F};
    bool is_valid{false};
};

// ============================================================================
// ENUMS CHẾ ĐỘ ĐỊNH VỊ & NHIỆM VỤ SINH TỒN
// ============================================================================
enum class NavigationMode : uint8_t {
    kGpsAided = 0,       // Định vị có GPS chính xác cao (<10m)
    kPdrAided = 1,       // Mất GPS, duy trì bằng PDR quán tính
    kTrnAided = 2,       // Khớp địa hình TRN Particle Filter
    kDegraded = 3,       // Suy giảm độ tin cậy (địa hình phẳng / nhiễu la bàn)
    kLost = 4,           // Mất phương hướng hoàn toàn, bất định lớn
    kEmergency = 5       // Kích hoạt cứu nạn SOS khẩn cấp
};

enum class MissionState : uint8_t {
    kNormal = 0,
    kPreAlert = 1,              // Phát hiện ngã / chấn động, đếm ngược 45s
    kDistressLocked = 2,        // Khóa trạng thái cứu hộ SOS khẩn cấp
    kPanicCircling = 3,         // Cảnh báo đi vòng tròn mất phương hướng
    kPanicCirclingWarning = 3,  // Alias tương thích ngược
    kRescueRequired = 4,        // Yêu cầu đội cứu hộ tiếp cận
    kEstimatorDegraded = 5      // Alias tương thích ngược
};

enum class PositionSource : uint8_t {
    kGps = 0,
    kPdr = 1,
    kTrn = 2,
    kGpsPdr = 3,
    kGpsTrn = 4,
    kPdrTrn = 5,
    kLastKnownGood = 6
};

// Alias tương thích ngược
using SafetyState = MissionState;

enum class PowerMode : uint8_t {
    kBalanced = 0,             // SoC > 20%: Chạy đầy đủ 10Hz PF
    kEnergyConservative = 1,   // 5% < SoC <= 20%: Giảm tần số PF 2Hz, tắt camera
    kCriticalSnapshot = 2,     // 2% < SoC <= 5%: Đóng băng dữ liệu, nạp outbound queue
    kDisplayOnly = 3           // SoC <= 2%: Khóa cảm biến, chỉ giữ màn hình cứu hộ
};

// ============================================================================
// PHÂN LOẠI HOẠT ĐỘNG CHUYỂN ĐỘNG & PHƯƠNG TIỆN (Motion & Activity Types)
// ============================================================================
enum class MotionActivity : uint8_t {
    kStationary = 0,
    kWalkingFlat = 1,
    kRunning = 2,
    kStairsAscent = 3,
    kStairsDescent = 4,
    kStairLanding = 5,
    kEscalator = 6,
    kCycling = 7,
    kVehicleWheeled = 8,
    kElevator = 9,
    kCableTransport = 10,
    kUnknown = 255
};

enum class VehicleSubtype : uint8_t {
    kUnknown = 0,
    kBicycle = 1,
    kMotorcycle = 2,
    kCar = 3,
    kBus = 4,
    kTrain = 5
};

struct StairState {
    uint32_t step_count{0};
    uint32_t landing_count{0};
    float estimated_riser_m{0.165F};
    float cumulative_vertical_m{0.0F};
    float cadence_hz{0.0F};
    float confidence{0.0F};
    bool is_on_landing{false};
};

struct VehicleState {
    VehicleSubtype subtype{VehicleSubtype::kUnknown};
    bool is_stopped{false};
    float gnss_speed_m_s{0.0F};
    float hf_vibration_ratio{0.0F};
    float confidence{0.0F};
};

struct MotionEvidence {
    float cadence_hz{0.0F};
    float vertical_energy{0.0F};
    float horizontal_energy{0.0F};
    float impact_score{0.0F};
    float freefall_score{0.0F};
    float hf_vibration_ratio{0.0F};
    float spectral_entropy{0.0F};
    float baro_vertical_rate{0.0F};
    float gnss_speed{0.0F};
    float stair_score{0.0F};
    float vehicle_score{0.0F};
    float escalator_score{0.0F};
    float elevator_score{0.0F};
};

struct MotionActivityEstimate {
    MotionActivity current_activity{MotionActivity::kStationary};
    MotionActivity candidate_activity{MotionActivity::kStationary};
    float raw_confidence{0.0F};
    float temporal_confidence{0.0F};
    float fused_confidence{0.0F};
    StairState stair_state{};
    VehicleState vehicle_state{};
    MotionEvidence evidence{};
};

// ============================================================================
// MÔ HÌNH SỨC KHỎE CẢM BIẾN & TÍNH TOÀN VẸN
// ============================================================================
struct SensorHealthModel {
    float imu{1.0F};
    float gyro{1.0F};
    float accel{1.0F};
    float magnetometer{1.0F};
    float barometer{1.0F};
    float gps{1.0F};
};

struct IntegrityStatus {
    bool is_healthy{true};
    bool position_valid{true};
    bool attitude_valid{true};
    bool velocity_valid{true};
    bool covariance_valid{true};
    double consistency_score{1.0};
    uint32_t fault_flags{0};
};

// ============================================================================
// TRẠNG THÁI CHÍNH TẮC THỐNG NHẤT (Canonical Navigation State)
// ============================================================================
struct CanonicalNavigationState {
    // 1. Vị trí và Chuyển động
    EastingNorthingAlt position_enu{};
    GeodeticCoord position_wgs84{};
    Vector3d velocity_m_s{};
    Vector3d instantaneous_velocity_m_s{};
    double speed_m_s{0.0};
    double vertical_speed_m_s{0.0};
    Quaterniond attitude_q{};
    Vector3d gyro_bias{};
    Vector3d accel_bias{};

    // 2. Hiệp phương sai đầy đủ
    std::array<double, 4> position_cov_2d{1.0, 0.0, 0.0, 1.0}; // [var_e, cov_en, cov_en, var_n]
    std::array<double, 9> attitude_cov_3x3{};
    double ce50_m{1.0};
    double ce95_m{2.45};
    double ellipse_heading_rad{0.0};

    // 3. Chế độ, Hoạt động và Nguồn gốc dữ liệu
    NavigationMode nav_mode{NavigationMode::kGpsAided};
    MissionState mission_state{MissionState::kNormal};
    PositionSource position_source{PositionSource::kGps};
    MotionActivityEstimate motion_activity{};
    double position_age_s{0.0};
    uint64_t timestamp_ns{0};

    // 4. Điểm chất lượng và Sức khỏe
    double global_navigation_score{1.0};
    SensorHealthModel sensor_health{};
    IntegrityStatus integrity{};

    // 5. Tái hiện và Kiểm toán
    uint64_t config_hash{0};
    uint64_t rng_seed{0};
};

} // namespace survival
