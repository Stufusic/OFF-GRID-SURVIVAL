#pragma once

#include <cstdint>
#include <vector>
#include <random>
#include <optional>
#include <array>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/environment/jean_meeus_solver.hpp"
#include "survival_core/estimators/particle_types.hpp"
#include "survival_core/math/static_spatial_hash.hpp"

namespace survival::estimators {

class ParticleFilterEngine {
public:
    explicit ParticleFilterEngine(const ParticleFilterConfig& config = ParticleFilterConfig{},
                                 uint32_t random_seed = 12345) noexcept;

    /**
     * @brief Khởi tạo đám mây hạt quanh vị trí ban đầu
     */
    void initialize(const EastingNorthingAlt& initial_pos, double pos_std_m = 15.0) noexcept;

    /**
     * @brief Lan truyền hạt theo sự kiện bước chân (Motion Update / Propagation)
     */
    void predict(const StepEvent& step) noexcept;

    /**
     * @brief Cập nhật trọng số hạt trong không gian Log-Likelihood dựa trên DEM và thiên văn
     * Áp dụng Log-Sum-Exp Trick và bảo vệ suy biến địa hình phẳng
     */
    void updateWeights(const environment::DemMap& dem,
                       double current_baro_alt_m,
                       double pdr_slope_rad,
                       double pdr_heading_rad,
                       std::optional<double> obs_sun_azimuth_rad = std::nullopt,
                       uint64_t timestamp_ns = 0) noexcept;

    /**
     * @brief Tái lấy mẫu phương sai thấp (Low-Variance Systematic Resampling)
     */
    void resample() noexcept;

    /**
     * @brief Trích xuất trạng thái độ bất định đa giả thuyết kèm phân cụm không gian nhanh
     */
    [[nodiscard]] NavigationUncertaintyState extractUncertaintyState(uint64_t timestamp_ns) const noexcept;

    [[nodiscard]] double getEffectiveParticleCount() const noexcept {
        return n_eff_;
    }

    [[nodiscard]] const std::vector<Particle>& getParticles() const noexcept {
        return particles_;
    }

    [[nodiscard]] EastingNorthingAlt getWeightedMeanPosition() const noexcept;

    [[nodiscard]] std::array<double, 4> getWeightedCovariance2D() const noexcept;

    [[nodiscard]] size_t getParticleCount() const noexcept {
        return particles_.size();
    }

    /**
     * @brief Tịnh tiến toàn bộ đám mây hạt theo delta (Bảo toàn hình thái phân phối xác suất khi handoff)
     */
    void shiftParticles(double delta_easting_m, double delta_northing_m, double delta_altitude_m) noexcept;

    /**
     * @brief Lan truyền đám mây hạt liên tục theo động học phương tiện (Stochastic Motion Covariance)
     */
    void propagateContinuousMotion(
        double delta_e_m, double delta_n_m, double delta_alt_m,
        double speed_m_s, double dt_s,
        MotionActivity activity) noexcept;

    /**
     * @brief Làm mịn đám mây hạt sau tái lấy mẫu thích ứng theo ESS và độ dốc địa hình
     */
    void applyAdaptiveKernelRoughening(
        double terrain_slope_norm,
        double pre_resample_neff = -1.0,
        const environment::DemMap* dem = nullptr
    ) noexcept;

    void setTargetParticleCount(size_t count) noexcept;

    void setRandomSeed(uint32_t seed) noexcept {
        rng_.seed(seed);
    }

private:
    ParticleFilterConfig config_{};
    std::vector<Particle> particles_{};
    std::vector<Particle> resample_buffer_{}; // Bộ đệm cấp phát sẵn để tái lấy mẫu O(N) không allocation

    // Bảng băm không gian tĩnh 512 cấp phát sẵn (Single-Thread Confined, tránh 32KB stack frame)
    mutable math::StaticSpatialHash<512> spatial_hash_{};

    double n_eff_{0.0};
    mutable std::mt19937 rng_;
};

} // namespace survival::estimators
