#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_core/math/frame_conventions.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace survival::estimators {

namespace {

// Ma trận Skew-symmetric [v x]
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) noexcept {
    Eigen::Matrix3d S;
    S <<  0.0,  -v.z(),  v.y(),
          v.z(),   0.0, -v.x(),
         -v.y(),  v.x(),   0.0;
    return S;
}

} // namespace

Eskf3D::Eskf3D(const EskfConfig& config) noexcept
    : config_(config)
{
    reset();
}

void Eskf3D::reset(const Quaterniond& initial_q, const Vector3d& initial_bias) noexcept {
    q_nominal_ = initial_q;
    q_nominal_.normalize();
    bg_nominal_ = initial_bias;
    last_gyro_ = {0.0F, 0.0F, 0.0F};
    last_timestamp_ns_ = 0;
    filtered_accel_ = Vector3d{0.0, 0.0, kGravityStandard};
    gravity_ref_ = Vector3d{0.0, 0.0, kGravityStandard};
    has_gravity_ref_ = false;
    last_predict_dt_s_ = 0.01;
    frames_since_reset_ = 0;
    is_last_quasi_static_ = false;
    is_last_mag_anomalous_ = false;
    last_mag_nis_ = 0.0;
    mag_health_ = 1.0F;
    imu_health_ = 1.0F;
    consecutive_mag_rejects_ = 0;

    // Khởi tạo ma trận hiệp phương sai 6x6
    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
    P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.01;  // Hiệp phương sai góc ban đầu ~ (0.1 rad)^2
    P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.001; // Hiệp phương sai bias ban đầu

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
        }
    }
}

void Eskf3D::predict(const Vector3f& gyro_rad_s, double dt_s) noexcept {
    if (dt_s <= 0.0 || dt_s > 0.5) {
        return; // Bỏ qua bước thời gian bất thường
    }

    last_gyro_ = gyro_rad_s;
    last_predict_dt_s_ = dt_s;
    frames_since_reset_++;

    // 1. Tích phân vận tốc góc không thiên vị: omega = omega_m - b_g
    const Eigen::Vector3d omega_unbiased(
        static_cast<double>(gyro_rad_s.x) - bg_nominal_.x,
        static_cast<double>(gyro_rad_s.y) - bg_nominal_.y,
        static_cast<double>(gyro_rad_s.z) - bg_nominal_.z
    );

    const double omega_norm = omega_unbiased.norm();
    Quaterniond dq{1.0, 0.0, 0.0, 0.0};

    if (omega_norm > 1e-8) {
        const double half_angle = 0.5 * omega_norm * dt_s;
        const double sin_factor = std::sin(half_angle) / omega_norm;
        dq.w = std::cos(half_angle);
        dq.x = omega_unbiased.x() * sin_factor;
        dq.y = omega_unbiased.y() * sin_factor;
        dq.z = omega_unbiased.z() * sin_factor;
    } else {
        dq.w = 1.0;
        dq.x = 0.5 * omega_unbiased.x() * dt_s;
        dq.y = 0.5 * omega_unbiased.y() * dt_s;
        dq.z = 0.5 * omega_unbiased.z() * dt_s;
    }

    q_nominal_ = math::quaternionMultiply(q_nominal_, dq);
    q_nominal_.normalize();

    // 2. Lan truyền Ma trận Hiệp phương sai Sai số (Error-State Propagation)
    // F_x = [ I - [omega x]*dt,  -I*dt ]
    //       [        0        ,    I   ]
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3, 3>(0, 0) -= skewSymmetric(omega_unbiased) * dt_s;
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * dt_s;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    const double q_gyro = config_.gyro_noise_std * config_.gyro_noise_std * dt_s;
    const double q_bias = config_.gyro_bias_rw_std * config_.gyro_bias_rw_std * dt_s;
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q_gyro;
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_bias;

    // Đọc P hiện tại
    Eigen::Matrix<double, 6, 6> P;
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            P(r, c) = p_cov_[static_cast<size_t>(r * 6 + c)];
        }
    }

    P = F * P * F.transpose() + Q;

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
        }
    }

    enforceSymmetryAndPsd();
}

bool Eskf3D::updateAccelerometer(const Vector3f& accel_m_s2) noexcept {
    // Lọc thông thấp vector trọng lực
    filtered_accel_.x = 0.95 * filtered_accel_.x + 0.05 * static_cast<double>(accel_m_s2.x);
    filtered_accel_.y = 0.95 * filtered_accel_.y + 0.05 * static_cast<double>(accel_m_s2.y);
    filtered_accel_.z = 0.95 * filtered_accel_.z + 0.05 * static_cast<double>(accel_m_s2.z);

    const Eigen::Vector3d a_raw(static_cast<double>(accel_m_s2.x),
                                static_cast<double>(accel_m_s2.y),
                                static_cast<double>(accel_m_s2.z));

    // [C1] Bộ lọc gravity tham chiếu chậm (dt-điều chế, τ = tilt_ref_tau_s):
    // mean gia tốc riêng theo một sải bước ≈ trọng lực kể cả khi đang đi bộ,
    // nên tham chiếu này hội tụ về hướng gravity THẬT ngay trong lúc chuyển động.
    {
        const Eigen::Vector3d g_ref_prev(gravity_ref_.x, gravity_ref_.y, gravity_ref_.z);
        const Eigen::Vector3d g_ref_new = has_gravity_ref_
            ? g_ref_prev + (a_raw - g_ref_prev) * (1.0 - std::exp(-last_predict_dt_s_ / config_.tilt_ref_tau_s))
            : a_raw;
        gravity_ref_ = Vector3d{g_ref_new.x(), g_ref_new.y(), g_ref_new.z()};
        has_gravity_ref_ = true;
    }

    const Eigen::Vector3d a_body(filtered_accel_.x, filtered_accel_.y, filtered_accel_.z);
    const double accel_norm = a_body.norm();
    const Eigen::Vector3d omega_unbiased(
        static_cast<double>(last_gyro_.x) - bg_nominal_.x,
        static_cast<double>(last_gyro_.y) - bg_nominal_.y,
        static_cast<double>(last_gyro_.z) - bg_nominal_.z
    );

    // 1. Kiểm tra Điều kiện tĩnh (Quasi-Static Gravity Gate)
    const bool is_static_accel = std::abs(accel_norm - kGravityStandard) < config_.static_accel_tol_m_s2;
    const bool is_static_gyro = omega_unbiased.norm() < config_.static_gyro_tol_rad_s;
    is_last_quasi_static_ = (is_static_accel && is_static_gyro);

    if (!is_last_quasi_static_ || accel_norm < 1e-3) {
        // [C1] Nhánh hiệu chỉnh tilt LIÊN TỤC (motion-tolerant complementary):
        // khi đang chuyển động, tilt không được phép kẹt ở giá trị khởi tạo.
        // Chỉ áp dụng khi tham chiếu gravity LP đáng tin (||a_lp|| ≈ g) và
        // tốc độ quay vừa phải (quay gắt thì đợi, tránh nuốt gia tốc động học).
        const double ref_norm = gravity_ref_.norm();
        const bool ref_valid = std::abs(ref_norm - kGravityStandard) < config_.tilt_ref_dev_tol_m_s2;
        const double omega_norm = omega_unbiased.norm();
        const double motion_gain = std::clamp(1.0 - omega_norm / config_.tilt_gyro_sat_rad_s, 0.0, 1.0);
        if (ref_valid && motion_gain > 0.0) {
            const Eigen::Matrix3d R_nb = math::quaternionToRotationMatrix(q_nominal_);
            const Eigen::Vector3d g_nav(0.0, 0.0, kGravityStandard);
            const Eigen::Vector3d g_b = R_nb.transpose() * g_nav;
            const Eigen::Vector3d g_ref(gravity_ref_.x, gravity_ref_.y, gravity_ref_.z);
            const Eigen::Vector3d residual = g_ref - g_b;
            // δθ = (r × g_b)/g² : quay tối thiểu đưa g_b dự đoán về hướng a quan sát
            Eigen::Vector3d delta_theta = (residual.cross(g_b)) / (kGravityStandard * kGravityStandard);
            const double warm_boost = (frames_since_reset_ < 300) ? 3.0 : 1.0; // hội tụ nhanh ~3s đầu
            delta_theta *= (config_.tilt_alpha * motion_gain * warm_boost);
            delta_theta.z() = 0.0; // Yaw không quan sát được từ trọng lực
            const Quaterniond dq{
                .w = 1.0,
                .x = 0.5 * delta_theta.x(),
                .y = 0.5 * delta_theta.y(),
                .z = 0.0
            };
            q_nominal_ = math::quaternionMultiply(q_nominal_, dq);
            q_nominal_.normalize();
        }
        return false; // Hợp đồng: không phải quasi-static -> không dùng EKF đầy đủ
    }

    // 2. Dự đoán trọng lực trong Body Frame: g_b = R_nb^T * [0, 0, g]^T
    const Eigen::Matrix3d R_nb = math::quaternionToRotationMatrix(q_nominal_);
    const Eigen::Vector3d g_nav(0.0, 0.0, kGravityStandard);
    const Eigen::Vector3d g_b = R_nb.transpose() * g_nav;

    const Eigen::Vector3d residual = a_body - g_b;

    // Measurement Jacobian H = [ [g_b x], 0_3x3 ] (3x6)
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = skewSymmetric(g_b);

    const double r_var = config_.accel_noise_std * config_.accel_noise_std;
    const Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * r_var;

    Eigen::Matrix<double, 6, 6> P;
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            P(r, c) = p_cov_[static_cast<size_t>(r * 6 + c)];
        }
    }

    const Eigen::Matrix3d S = H * P * H.transpose() + R_meas;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    Eigen::Matrix<double, 6, 1> delta_x = K * residual;
    delta_x(2) = 0.0; // Trục Yaw không quan sát được từ trọng lực
    delta_x(5) = 0.0; // Bias Gyro Z không quan sát được từ trọng lực

    // 3. Joseph-Form Covariance Update
    const Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * R_meas * K.transpose();

    // 4. Bơm sai số vào trạng thái danh định
    const Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
    const Eigen::Vector3d delta_bg = delta_x.segment<3>(3);

    const Quaterniond dq{
        .w = 1.0,
        .x = 0.5 * delta_theta.x(),
        .y = 0.5 * delta_theta.y(),
        .z = 0.0
    };
    q_nominal_ = math::quaternionMultiply(q_nominal_, dq);
    q_nominal_.normalize();

    bg_nominal_.x += delta_bg.x();
    bg_nominal_.y += delta_bg.y();
    bg_nominal_.z += delta_bg.z();

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
        }
    }

    enforceSymmetryAndPsd();
    return true;
}

bool Eskf3D::updateMagnetometer(const Vector3f& mag_uT) noexcept {
    const Eigen::Vector3d m_body(
        static_cast<double>(mag_uT.x),
        static_cast<double>(mag_uT.y),
        static_cast<double>(mag_uT.z)
    );

    const double mag_norm = m_body.norm();

    // 1. Kiểm tra dải cường độ tổng
    if (mag_norm < config_.mag_norm_min_uT || mag_norm > config_.mag_norm_max_uT) {
        is_last_mag_anomalous_ = true;
        last_mag_nis_ = 25.0;
        consecutive_mag_rejects_++;
        mag_health_ = std::max(0.0F, mag_health_ - 0.15F);
        return false;
    }


    // 2. Chiếu trực tiếp sang Navigation Frame qua DCM R_nb: m_nav = R_nb * m_body
    const Eigen::Matrix3d R_nb = math::quaternionToRotationMatrix(q_nominal_);
    const Eigen::Vector3d m_nav = R_nb * m_body;

    // Trong hệ ENU, vectơ từ trường Trái Đất hướng về phương Bắc từ (+Y)
    // Sai lệch góc hướng Yaw: atan2(-m_nav.x(), m_nav.y())
    // [B3] Bù từ sai (declination, Đông-dương): không bù thì bộ lọc hội tụ về
    // BẮC TỪ — heading ước lượng lệch cả góc D so với Bắc thật.
    double innovation_yaw = std::atan2(-m_nav.x(), m_nav.y()) + config_.mag_declination_rad;
    innovation_yaw = std::atan2(std::sin(innovation_yaw), std::cos(innovation_yaw));  // [-pi, pi]

    // 3. Measurement Jacobian H_mag = [0, 0, -1, 0, 0, 0] (1x6)
    // Vì psi = psi_0 - delta_theta_z (quay clockwise quanh +Z)
    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H(0, 2) = -1.0;

    const double r_nominal = config_.mag_heading_noise_std * config_.mag_heading_noise_std;
    const double r_eff = r_nominal / (static_cast<double>(mag_health_) + 1e-4);

    Eigen::Matrix<double, 6, 6> P;
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            P(r, c) = p_cov_[static_cast<size_t>(r * 6 + c)];
        }
    }

    const double S = P(2, 2) + r_eff;
    if (S < 1e-12) {
        return false;
    }

    const double NIS = (innovation_yaw * innovation_yaw) / S;
    last_mag_nis_ = NIS;

    // 4. Chi-Square Innovation Gating (3-sigma = 9.0)
    if (NIS > config_.mag_max_nis) {
        is_last_mag_anomalous_ = true;
        consecutive_mag_rejects_++;
        mag_health_ = std::max(0.0F, mag_health_ - 0.10F);
        return false;
    }

    // Gating PASS
    is_last_mag_anomalous_ = false;
    const bool recovering_from_disturbance = (consecutive_mag_rejects_ > 0);
    consecutive_mag_rejects_ = 0;
    // [C3b] Hồi phục nhanh giờ AN TOÀN vì update đã heading-pure (K projection
    // bên dưới) — innovation heading không còn kéo tilt qua coupling P.
    mag_health_ = std::min(1.0F, mag_health_ + (recovering_from_disturbance ? 0.06F : 0.02F));

    Eigen::Matrix<double, 6, 1> K = -P.col(2) / S; // K = P * H^T / S
    // [C3b] Heading-pure update: phép đo YAW chỉ được hiệu chỉnh YAW (state 2)
    // và bias gyro-Z (state 5). Bản cũ để K đầy đủ — nhiễu từ kéo tilt/qua
    // coupling P tạo wobble roll/pitch, làm rung nhãn thang (thử nghiệm D1).
    K(0) = 0.0;
    K(1) = 0.0;
    K(3) = 0.0;
    K(4) = 0.0;
    const Eigen::Matrix<double, 6, 1> delta_x = K * innovation_yaw;

    // Joseph-Form Update
    const Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * r_eff * K.transpose();

    // Bơm sai số và Reset
    const Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
    const Eigen::Vector3d delta_bg = delta_x.segment<3>(3);

    const Quaterniond dq{
        .w = 1.0,
        .x = 0.5 * delta_theta.x(),
        .y = 0.5 * delta_theta.y(),
        .z = 0.5 * delta_theta.z()
    };
    q_nominal_ = math::quaternionMultiply(q_nominal_, dq);
    q_nominal_.normalize();

    bg_nominal_.x += delta_bg.x();
    bg_nominal_.y += delta_bg.y();
    bg_nominal_.z += delta_bg.z();

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
        }
    }

    enforceSymmetryAndPsd();
    return true;
}

bool Eskf3D::updateZeroVelocity(double dt_s) noexcept {
    if (dt_s <= 0.0) return false;
    return true;
}

bool Eskf3D::updateZeroAngularRate() noexcept {
    if (!is_last_quasi_static_) return false;
    // [C3] Chỉ hiệu chuẩn khi TĨNH SÂU (|ω − bg| < 0.02 rad/s): quasi-static rộng
    // 0.05 vẫn chứa drift tay chậm — hấp thụ drift vào bias làm lệch heading sau đó.
    const Eigen::Vector3d omega_res(
        static_cast<double>(last_gyro_.x) - bg_nominal_.x,
        static_cast<double>(last_gyro_.y) - bg_nominal_.y,
        static_cast<double>(last_gyro_.z) - bg_nominal_.z);
    if (omega_res.norm() > 0.02) {
        return false;
    }
    bg_nominal_.x += 0.02 * omega_res.x();
    bg_nominal_.y += 0.02 * omega_res.y();
    bg_nominal_.z += 0.02 * omega_res.z();
    return true;
}

void Eskf3D::processSensorFrame(const SensorFrame& frame, double dt_s) noexcept {
    last_timestamp_ns_ = frame.timestamp_ns;

    if (frame.has_imu) {
        Vector3f gyro{frame.gyro_rad_s[0], frame.gyro_rad_s[1], frame.gyro_rad_s[2]};
        Vector3f accel{frame.accel_m_s2[0], frame.accel_m_s2[1], frame.accel_m_s2[2]};

        predict(gyro, dt_s);
        updateAccelerometer(accel);
    }

    if (frame.has_mag) {
        Vector3f mag{frame.mag_uT[0], frame.mag_uT[1], frame.mag_uT[2]};
        updateMagnetometer(mag);
    }
}

Vector3d Eskf3D::rotateBodyToNav(const Vector3f& v_body) const noexcept {
    const Eigen::Matrix3d R_nb = math::quaternionToRotationMatrix(q_nominal_);
    const Eigen::Vector3d v_b(
        static_cast<double>(v_body.x),
        static_cast<double>(v_body.y),
        static_cast<double>(v_body.z)
    );
    const Eigen::Vector3d v_n = R_nb * v_b;
    return Vector3d{v_n.x(), v_n.y(), v_n.z()};
}

double Eskf3D::getVerticalAcceleration(const Vector3f& accel_m_s2) const noexcept {
    const Vector3d a_nav = rotateBodyToNav(accel_m_s2);
    return a_nav.z - kGravityStandard;
}

double Eskf3D::getHeadingRad() const noexcept {
    return math::extractCompassHeadingRad(q_nominal_);
}

AttitudeEstimate Eskf3D::getAttitudeEstimate() const noexcept {
    AttitudeEstimate est{};
    est.timestamp_ns = last_timestamp_ns_;
    est.quaternion = q_nominal_;
    est.gyro_bias_rad_s = bg_nominal_;

    const double w = q_nominal_.w;
    const double x = q_nominal_.x;
    const double y = q_nominal_.y;
    const double z = q_nominal_.z;

    // Frame BODY (docs/frame_conventions.md §II): +X = Phải, +Y = Trước, +Z = Lên.
    // Ngữ nghĩa euler theo attitude_types.hpp: roll = nghiêng NGANG, pitch = đổ VỀ TRƯỚC.
    // Roll (bank quanh trục trước +Y): roll = atan2(−R(2,0), R(2,2)); dương = nghiêng phải.
    const double sinr = 2.0 * (w * y - x * z);              // −R(2,0)
    const double cosr = 1.0 - 2.0 * (x * x + y * y);        //  R(2,2)
    est.euler.roll_rad = std::atan2(sinr, cosr);

    // Pitch (đổ về trước quanh trục phải +X): pitch = −asin(R(2,1)); dương = trục trước chúc xuống.
    const double fwd_elev_sin = 2.0 * (y * z + w * x);      //  R(2,1)
    if (std::abs(fwd_elev_sin) >= 1.0) {
        est.euler.pitch_rad = std::copysign(kHalfPi, -fwd_elev_sin);
    } else {
        est.euler.pitch_rad = -std::asin(fwd_elev_sin);
    }

    // Yaw (psi)
    est.euler.yaw_rad = getHeadingRad();

    // Trích xuất ma trận 3x3 hiệp phương sai góc
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            est.attitude_cov_3x3[static_cast<size_t>(r * 3 + c)] = p_cov_[static_cast<size_t>(r * 6 + c)];
        }
    }

    est.heading_variance_rad2 = p_cov_[static_cast<size_t>(2 * 6 + 2)];
    est.is_quasi_static = is_last_quasi_static_;
    est.is_mag_anomalous = is_last_mag_anomalous_;

    return est;
}

void Eskf3D::enforceSymmetryAndPositiveDefiniteness() noexcept {
    Eigen::Matrix<double, 6, 6> P;
    bool has_nan_or_inf = false;
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            const double val = p_cov_[static_cast<size_t>(r * 6 + c)];
            if (!std::isfinite(val)) {
                has_nan_or_inf = true;
            }
            P(r, c) = val;
        }
    }

    if (has_nan_or_inf) {
        // Khôi phục khẩn cấp khi gặp giá trị không hữu hạn
        P = 1e-2 * Eigen::Matrix<double, 6, 6>::Identity();
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
            }
        }
        return;
    }

    // Bước 1: Đối xứng hóa ma trận
    P = 0.5 * (P + P.transpose());

    // Bước 2: Thử nghiệm phân tích Cholesky trực tiếp
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(P);
    if (llt.info() == Eigen::Success && P.diagonal().minCoeff() >= 1e-8) {
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
            }
        }
        return;
    }

    // Bước 3: Fallback phân tích phổ trị riêng (Eigenvalue Decomposition)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(P);
    if (es.info() == Eigen::Success) {
        const double max_lambda = es.eigenvalues().maxCoeff();
        const double eps_floor = std::max(1e-8, 1e-6 * max_lambda);

        Eigen::Matrix<double, 6, 1> clamped_lambdas = es.eigenvalues();
        for (int i = 0; i < 6; ++i) {
            if (clamped_lambdas(i) < eps_floor) {
                clamped_lambdas(i) = eps_floor;
            }
        }

        P = es.eigenvectors() * clamped_lambdas.asDiagonal() * es.eigenvectors().transpose();
        P = 0.5 * (P + P.transpose());

        // Bước 4: Tái kiểm tra Cholesky sau tái tạo
        Eigen::LLT<Eigen::Matrix<double, 6, 6>> re_llt(P);
        if (re_llt.info() == Eigen::Success) {
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
                }
            }
            return;
        }

        // Thêm số hạng điều hòa Tikhonov nếu cần
        P += 1e-7 * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::LLT<Eigen::Matrix<double, 6, 6>> tik_llt(P);
        if (tik_llt.info() == Eigen::Success) {
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
                }
            }
            return;
        }
    }

    // Bước 5: Ultimate Fallback - Khởi tạo lại hiệp phương sai an toàn
    P = 1e-2 * Eigen::Matrix<double, 6, 6>::Identity();
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            p_cov_[static_cast<size_t>(r * 6 + c)] = P(r, c);
        }
    }
}

} // namespace survival::estimators
