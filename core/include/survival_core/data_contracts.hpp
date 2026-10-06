#pragma once

#include <cstdint>
#include <array>
#include <span>
#include "survival_core/types.hpp"

namespace survival {

// ============================================================================
// 1. GÓI DỮ LIỆU CẢM BIẾN ĐẦU VÀO (Multi-Rate Sensor Frame)
// ============================================================================
struct alignas(16) SensorFrame {
    uint64_t timestamp_ns{0};
    
    // Dữ liệu IMU (100 Hz)
    std::array<float, 3> accel_m_s2{0.0F, 0.0F, 0.0F};
    std::array<float, 3> gyro_rad_s{0.0F, 0.0F, 0.0F};
    
    // Dữ liệu Từ kế (20 Hz)
    std::array<float, 3> mag_uT{0.0F, 0.0F, 0.0F};
    
    // Dữ liệu Áp kế (10 Hz)
    float pressure_hpa{1013.25F};
    
    // Dữ liệu Thiên văn / Camera (Event-driven)
    float optical_sun_azimuth_rad{-1.0F}; // Giá trị < 0 nếu không có quan sát
    float optical_confidence{0.0F};

    // Cờ báo hiệu tính khả dụng đa tần số (Multi-Rate Availability Flags)
    bool has_imu{true};
    bool has_mag{true};
    bool has_baro{true};
    bool has_optical_sun{false};

    // [B4] Vận tốc GNSS Doppler [m/s]; < 0 = không có GNSS. Dùng làm tốc độ đầu vào
    // cho classifier (khởi động cycling/cable gate) và velocity estimator.
    float gnss_speed_m_s{-1.0F};
};

// ============================================================================
// 2. SỰ KIỆN BƯỚC CHÂN PDR (Step Event)
// ============================================================================
struct StepEvent {
    uint64_t timestamp_ns{0};
    double stride_mean_m{0.75};           // [m] Kỳ vọng chiều dài bước
    double stride_variance_m2{0.0025};    // [m^2] Phương sai chiều dài bước
    double heading_rad{0.0};              // [rad] Góc hướng bước chân
    double heading_variance_rad2{0.01};   // [rad^2] Phương sai góc hướng
    double vertical_displacement_m{0.0};  // [m] Độ dịch chuyển cao độ (từ áp kế hoặc ước lượng bậc thang)
    double confidence{1.0};               // Trọng số tin cậy bước [0.0 - 1.0]
    MotionActivity activity{MotionActivity::kWalkingFlat}; // Phân loại hoạt động tại bước chân
};

// ============================================================================
// 3. HÌNH HỌC BẤT ĐỊNH CỤM GIẢ THUYẾT (Hypothesis Geometry)
// ============================================================================
struct HypothesisGeometry {
    uint8_t cluster_id{0};
    double weight{0.0};                   // Trọng số phân bố xác suất [0.0 - 1.0]
    double center_easting_m{0.0};         // [m] Tọa độ tâm Easting
    double center_northing_m{0.0};        // [m] Tọa độ tâm Northing
    double center_altitude_m{0.0};        // [m] Cao độ tâm
    
    // Tham số Confidence Ellipse 95%
    double ce95_semi_major_m{0.0};        // [m] Bán trục lớn a_95
    double ce95_semi_minor_m{0.0};        // [m] Bán trục nhỏ b_95
    double ellipse_heading_rad{0.0};      // [rad] Góc xoay của elip so với hướng Bắc
    double area_95_m2{0.0};               // [m^2] Diện tích elip bất định 95%
};

// ============================================================================
// 4. TRẠNG THÁI ĐIỀU HƯỚNG TỔNG THỂ (Navigation Uncertainty State)
// ============================================================================
struct NavigationUncertaintyState {
    uint64_t timestamp_ms{0};
    uint32_t session_id{0};
    uint8_t boot_counter{0};
    uint8_t sequence_number{0};
    
    uint8_t active_clusters_count{1};
    std::array<HypothesisGeometry, 4> clusters{};
    
    double global_navigation_score{1.0};          // Điểm GNS [0.0 - 1.0]
    double calibrated_containment_probability{1.0};// Xác suất P(True in CE95)
    
    // Cờ trạng thái hệ thống và sức khỏe cảm biến (Bitfield)
    struct SystemFlags {
        bool imu_healthy : 1 {true};
        bool baro_healthy : 1 {true};
        bool mag_anomaly : 1 {false};
        bool celestial_available : 1 {false};
        bool panic_circling : 1 {false};
        bool immobility_warning : 1 {false};
        bool distress_sos_active : 1 {false};
        bool estimator_diverged : 1 {false};
        bool estimator_stale : 1 {false};
        bool zupt_active : 1 {false};
    } flags{};
    
    uint8_t battery_percent{100};
    MissionState safety_state{MissionState::kNormal};
    NavigationMode nav_mode{NavigationMode::kGpsAided};
    PositionSource position_source{PositionSource::kGps};
    PowerMode power_mode{PowerMode::kBalanced};
    double position_age_s{0.0};

    // Hoạt động chuyển động và Vận tốc tức thời 100Hz
    MotionActivity motion_activity{MotionActivity::kStationary};
    float activity_confidence{0.0F};
    float instantaneous_speed_m_s{0.0F};
    float vertical_speed_m_s{0.0F};
};

// ============================================================================
// 5. GÓI TIN CỨU HỘ 160-BIT (Outbound Rescue Packet 20 Bytes)
// ============================================================================
struct alignas(1) OutboundRescuePacket160 {
    std::array<uint8_t, 20> data{};

    [[nodiscard]] constexpr size_t size() const noexcept { return 20; }
    [[nodiscard]] const uint8_t* bytes() const noexcept { return data.data(); }
    [[nodiscard]] uint8_t* bytes() noexcept { return data.data(); }
    [[nodiscard]] uint8_t operator[](size_t idx) const noexcept { return data[idx]; }
    [[nodiscard]] uint8_t& operator[](size_t idx) noexcept { return data[idx]; }
};

static_assert(sizeof(OutboundRescuePacket160) == 20, "OutboundRescuePacket160 must be exactly 20 bytes!");

} // namespace survival
