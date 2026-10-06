#pragma once

#include <cstdint>
#include <array>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_core/estimators/attitude_types.hpp"

namespace survival::estimators {

struct EskfConfig {
    double gyro_noise_std{0.002};          // [rad/s] Mật độ nhiễu góc Gyro
    double gyro_bias_rw_std{1e-5};         // [rad/s^2] Random walk trôi bias Gyro
    double accel_noise_std{0.20};          // [m/s^2] Nhiễu gia tốc kế khi tĩnh
    double mag_heading_noise_std{0.04};    // [rad] Nhiễu góc phương vị từ kế
    
    // Ngưỡng kiểm định tĩnh Quasi-Static
    double static_accel_tol_m_s2{0.4};     // | ||a|| - g | < 0.4 m/s^2
    double static_gyro_tol_rad_s{0.05};    // ||w - b|| < 0.05 rad/s
    
    // Ngưỡng phát hiện dị thường từ trường
    double mag_norm_min_uT{20.0};
    double mag_norm_max_uT{70.0};
    double mag_max_nis{9.0};               // Ngưỡng Chi-square Innovation Gate (3-sigma = 9.0)
    
    // ZUPT / ZARU
    double zupt_noise_std{0.05};           // [m/s] Độ tin cậy giả định vận tốc 0 khi đứng yên

    // [B3] Từ sai (magnetic declination) [rad], Đông-dương. 0 = giả định Bắc từ = Bắc thật.
    double mag_declination_rad{0.0};

    // [C1] Hiệu chỉnh tilt LIÊN TỤC (chuyển động): tilt chỉ hội tụ qua nhánh
    // quasi-static là không đủ — điện thoại trong túi nghiêng tĩnh 45-65° và
    // người dùng đi liên tục khiến tilt kẹt ở khởi tạo, phá vỡ mọi trục đứng
    // phía hạ lưu (a_v_dynamic, delta-Z, hướng leo).
    double tilt_ref_tau_s{2.5};          // [s] Hằng số thời gian bộ lọc gravity tham chiếu chậm
    double tilt_ref_dev_tol_m_s2{2.0};   // [m/s^2] | ||a_lp|| - g | tối đa để chấp nhận tham chiếu
    double tilt_gyro_sat_rad_s{2.0};     // [rad/s] ||ω|| làm gain tilt liên tục về 0
    double tilt_alpha{0.02};             // Gain cơ bản mỗi khung (× motion_gain, warm-boost 3s đầu)
};

class Eskf3D {
public:
    explicit Eskf3D(const EskfConfig& config = EskfConfig{}) noexcept;

    /**
     * @brief Khởi tạo lại trạng thái thái độ ban đầu
     */
    void reset(const Quaterniond& initial_q = Quaterniond{1.0, 0.0, 0.0, 0.0},
               const Vector3d& initial_bias = Vector3d{0.0, 0.0, 0.0}) noexcept;

    /// [B3] Cập nhật từ sai tại runtime (vị trí thay đổi theo vĩ độ)
    void setMagDeclinationRad(double declination_rad) noexcept {
        config_.mag_declination_rad = declination_rad;
    }

    /**
     * @brief Bước Dự đoán (Nominal Kinematics + Error Covariance Propagation) ở nhịp 100 Hz
     */
    void predict(const Vector3f& gyro_rad_s, double dt_s) noexcept;

    /**
     * @brief Bước Hiệu chỉnh Gia tốc kế (Roll & Pitch Tilt Update) có Quasi-Static Gravity Gate
     */
    bool updateAccelerometer(const Vector3f& accel_m_s2) noexcept;

    /**
     * @brief Bước Hiệu chỉnh Từ kế (Heading / Yaw Update) dùng DCM chuẩn và Innovation Gating
     */
    bool updateMagnetometer(const Vector3f& mag_uT) noexcept;

    /**
     * @brief Zero-Velocity Update (ZUPT): Triệt tiêu trôi vận tốc trong pha chạm đất tĩnh
     */
    bool updateZeroVelocity(double dt_s) noexcept;

    /**
     * @brief Zero-Angular-Rate Update (ZARU): Hiệu chuẩn Gyro Bias khi tĩnh
     */
    bool updateZeroAngularRate() noexcept;

    /**
     * @brief Xử lý một khung dữ liệu SensorFrame hoàn chỉnh (Multi-Rate)
     */
    void processSensorFrame(const SensorFrame& frame, double dt_s) noexcept;

    /**
     * @brief Lấy trạng thái thái độ ước lượng hiện tại
     */
    [[nodiscard]] AttitudeEstimate getAttitudeEstimate() const noexcept;

    /**
     * @brief Xoay vectơ từ hệ quy chiếu thân máy (Body frame) sang hệ đạo hàng (ENU frame)
     * v_nav = R_nb * v_body
     */
    [[nodiscard]] Vector3d rotateBodyToNav(const Vector3f& v_body) const noexcept;

    /**
     * @brief Trích xuất gia tốc thẳng đứng thực tế (đã bù trừ trọng lực)
     * a_v = a_nav,z - g
     */
    [[nodiscard]] double getVerticalAcceleration(const Vector3f& accel_m_s2) const noexcept;

    [[nodiscard]] const Quaterniond& getQuaternion() const noexcept {
        return q_nominal_;
    }

    [[nodiscard]] const Vector3d& getGyroBias() const noexcept {
        return bg_nominal_;
    }

    [[nodiscard]] double getHeadingRad() const noexcept;

    [[nodiscard]] double getLatestMagNis() const noexcept {
        return last_mag_nis_;
    }

    [[nodiscard]] float getMagHealth() const noexcept {
        return mag_health_;
    }

    [[nodiscard]] float getImuHealth() const noexcept {
        return imu_health_;
    }

private:
    EskfConfig config_{};
    
    // Trạng thái danh định (Nominal state)
    Quaterniond q_nominal_{1.0, 0.0, 0.0, 0.0};
    Vector3d bg_nominal_{0.0, 0.0, 0.0};
    Vector3d filtered_accel_{0.0, 0.0, 9.80665};
    Vector3f last_gyro_{0.0F, 0.0F, 0.0F};
    uint64_t last_timestamp_ns_{0};

    // [C1] Tham chiếu gravity LP chậm cho hiệu chỉnh tilt liên tục
    Vector3d gravity_ref_{0.0, 0.0, 9.80665};
    bool has_gravity_ref_{false};
    double last_predict_dt_s_{0.01};
    uint32_t frames_since_reset_{0};

    // Ma trận hiệp phương sai sai số 6x6 (Row-Major flat array)
    // State error: [delta_theta (3), delta_bg (3)]
    std::array<double, 36> p_cov_{};

    bool is_last_quasi_static_{false};
    bool is_last_mag_anomalous_{false};
    double last_mag_nis_{0.0};
    float mag_health_{1.0F};
    float imu_health_{1.0F};
    uint32_t consecutive_mag_rejects_{0};

    void enforceSymmetryAndPositiveDefiniteness() noexcept;
    void enforceSymmetryAndPsd() noexcept {
        enforceSymmetryAndPositiveDefiniteness();
    }
};

} // namespace survival::estimators
