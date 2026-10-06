#include "survival_core/estimators/particle_filter_engine.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_core/math/statistics.hpp"
#include "survival_core/math/static_spatial_hash.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace survival::estimators {

ParticleFilterEngine::ParticleFilterEngine(const ParticleFilterConfig& config, uint32_t random_seed) noexcept
    : config_(config)
    , rng_(random_seed)
{
    particles_.resize(config_.num_particles);
    resample_buffer_.resize(config_.num_particles);
    n_eff_ = static_cast<double>(config_.num_particles);
}

void ParticleFilterEngine::initialize(const EastingNorthingAlt& initial_pos, double pos_std_m) noexcept {
    std::normal_distribution<double> dist_e(initial_pos.easting_m, pos_std_m);
    std::normal_distribution<double> dist_n(initial_pos.northing_m, pos_std_m);
    std::normal_distribution<double> dist_alt(initial_pos.altitude_m, 2.0);

    const double init_log_weight = -std::log(static_cast<double>(config_.num_particles));
    const double init_weight = 1.0 / static_cast<double>(config_.num_particles);

    for (size_t i = 0; i < config_.num_particles; ++i) {
        particles_[i].easting_m = dist_e(rng_);
        particles_[i].northing_m = dist_n(rng_);
        particles_[i].altitude_m = dist_alt(rng_);
        particles_[i].log_weight = init_log_weight;
        particles_[i].weight = init_weight;
        particles_[i].is_alive = true;
    }

    n_eff_ = static_cast<double>(config_.num_particles);
}

void ParticleFilterEngine::shiftParticles(double delta_easting_m, double delta_northing_m, double delta_altitude_m) noexcept {
    for (auto& p : particles_) {
        p.easting_m += delta_easting_m;
        p.northing_m += delta_northing_m;
        p.altitude_m += delta_altitude_m;
    }
}

void ParticleFilterEngine::predict(const StepEvent& step) noexcept {
    const double sigma_L = std::sqrt(std::max(1e-6, step.stride_variance_m2));
    const double sigma_psi = std::sqrt(std::max(1e-6, step.heading_variance_rad2));

    std::normal_distribution<double> dist_noise_L(0.0, sigma_L);
    std::normal_distribution<double> dist_noise_psi(0.0, sigma_psi);

    for (auto& p : particles_) {
        if (!p.is_alive) continue;

        const double perturbed_L = std::max(0.10, step.stride_mean_m + dist_noise_L(rng_));
        // [A1] Wrap circular: noise cộng thẳng vào radian cũ có thể lệch quá ±pi
        const double perturbed_psi = step.heading_rad
            + std::atan2(std::sin(dist_noise_psi(rng_)), std::cos(dist_noise_psi(rng_)));

        p.easting_m += perturbed_L * std::sin(perturbed_psi);
        p.northing_m += perturbed_L * std::cos(perturbed_psi);
        p.altitude_m += step.vertical_displacement_m;
    }
}

void ParticleFilterEngine::updateWeights(
    const environment::DemMap& dem,
    double current_baro_alt_m,
    double pdr_slope_rad,
    double pdr_heading_rad,
    std::optional<double> obs_sun_azimuth_rad,
    uint64_t timestamp_ns) noexcept
{
    const double var_elev = config_.sigma_elev_m * config_.sigma_elev_m;
    const double var_slope = config_.sigma_slope_rad * config_.sigma_slope_rad;
    const double var_aspect = config_.sigma_aspect_rad * config_.sigma_aspect_rad;
    const double var_sun = config_.sigma_sun_rad * config_.sigma_sun_rad;

    // 1. Cập nhật Log-Likelihood cho từng hạt
    for (auto& p : particles_) {
        if (!dem.isInside(p.easting_m, p.northing_m)) {
            // Hạt rơi ra ngoài biên DEM -> Loại trừ (Culling)
            p.is_alive = false;
            p.log_weight = -1e9;
            continue;
        }

        p.is_alive = true;
        const auto z_dem = dem.getElevationBilinear(p.easting_m, p.northing_m);
        if (!z_dem) {
            p.is_alive = false;
            p.log_weight = -1e9;
            continue;
        }

        // A. Likelihood cao độ áp kế vs DEM: ln p_elev = -0.5 * (z_baro - z_dem)^2 / sigma_e^2
        const double diff_z = current_baro_alt_m - *z_dem;
        const double ln_p_elev = -0.5 * (diff_z * diff_z) / var_elev;

        // B. Likelihood độ dốc PDR vs DEM: ln p_slope = -0.5 * (theta_pdr - theta_dem)^2 / sigma_s^2
        const auto grad = dem.getGradient(p.easting_m, p.northing_m);
        // [A1] Hạt gần mép DEM: getGradient trả is_valid=false (stencil chạm biên)
        // — bản cũ vẫn chấm slope/aspect/traversability với slope_rad=0 giả tạo,
        // tạo bằng chứng sai so với các hạt nội địa. Chỉ giữ likelihood cao độ.
        double ln_p_slope = 0.0;
        double ln_p_aspect = 0.0;
        double ln_p_trav = 0.0;
        if (grad.is_valid) {
            const double diff_slope = pdr_slope_rad - grad.slope_rad;
            ln_p_slope = -0.5 * (diff_slope * diff_slope) / var_slope;

            // C. Likelihood hướng sườn dốc (Aspect) kèm bảo vệ suy biến địa hình phẳng
            if (grad.gradient_norm >= config_.aspect_s_min) {
                // Khử lệch góc cuốn vòng
                const double delta_psi_circ = math::angleDifferenceRad(grad.aspect_rad, pdr_heading_rad);
                const double aspect_scale = std::tanh(grad.gradient_norm / config_.aspect_s0);
                ln_p_aspect = -0.5 * (delta_psi_circ * delta_psi_circ) / var_aspect * aspect_scale;
            }

            // D. Phạt mềm khả vi hành (Logistic Soft Traversability Gate khi slope > 55 độ = 0.96 rad)
            const double slope_excess = grad.slope_rad - config_.max_traversable_slope_rad;
            ln_p_trav = (slope_excess > 0.0) ? (-15.0 * slope_excess) : 0.0;
        }

        // E. Ràng buộc quan sát thiên văn Mặt Trời
        double ln_p_sun = 0.0;
        if (obs_sun_azimuth_rad.has_value() && timestamp_ns > 0) {
            const auto [lat, lon] = dem.enuToLatLon(p.easting_m, p.northing_m);
            const auto solar = environment::JeanMeeusSolver::computeSolarPosition(timestamp_ns, lat, lon);
            if (solar.is_daylight) {
                const double diff_sun = math::angleDifferenceRad(*obs_sun_azimuth_rad, solar.azimuth_rad);
                ln_p_sun = -0.5 * (diff_sun * diff_sun) / var_sun;
            }
        }

        // Tích hợp tổng Log-Likelihood
        p.log_weight += config_.lambda_elev * ln_p_elev +
                        config_.lambda_slope * ln_p_slope +
                        config_.lambda_aspect * ln_p_aspect +
                        config_.lambda_trav * ln_p_trav +
                        config_.lambda_sun * ln_p_sun;
        p.altitude_m = *z_dem; // Gán lại độ cao theo DEM
    }

    // 2. Chuẩn hóa Log-Sum-Exp Trick để triệt tiêu lỗi tràn số dưới
    double max_log_weight = -1e12;
    for (const auto& p : particles_) {
        if (p.is_alive && p.log_weight > max_log_weight) {
            max_log_weight = p.log_weight;
        }
    }

    if (max_log_weight < -1e8) {
        // Tất cả hạt đều bị culling hoặc suy biến hoàn toàn -> hồi sinh đều
        const double uniform_w = 1.0 / static_cast<double>(config_.num_particles);
        const double uniform_ln = -std::log(static_cast<double>(config_.num_particles));
        for (auto& p : particles_) {
            p.weight = uniform_w;
            p.log_weight = uniform_ln;
            p.is_alive = true;
        }
        n_eff_ = static_cast<double>(config_.num_particles);
        return;
    }

    double sum_unnorm_weight = 0.0;
    for (auto& p : particles_) {
        if (p.is_alive) {
            p.weight = std::exp(p.log_weight - max_log_weight);
            sum_unnorm_weight += p.weight;
        } else {
            p.weight = 0.0;
        }
    }

    if (sum_unnorm_weight > 0.0) {
        double sum_w_squared = 0.0;
        for (auto& p : particles_) {
            p.weight /= sum_unnorm_weight;
            p.log_weight = std::log(std::max(1e-300, p.weight));
            sum_w_squared += p.weight * p.weight;
        }
        n_eff_ = (sum_w_squared > 1e-12) ? (1.0 / sum_w_squared) : 1.0;
    } else {
        n_eff_ = 1.0;
    }

    // 3. Kích hoạt Tái lấy mẫu Systematic Resampling khi N_eff tụt quá ngưỡng
    if (n_eff_ < static_cast<double>(config_.num_particles) * config_.neff_resample_ratio) {
        const double pre_resample_neff = n_eff_;
        resample();
        applyAdaptiveKernelRoughening(std::abs(pdr_slope_rad), pre_resample_neff, &dem);
    }
}

void ParticleFilterEngine::applyAdaptiveKernelRoughening(
    double terrain_slope_norm,
    double pre_resample_neff,
    const environment::DemMap* dem
) noexcept {
    const size_t N = particles_.size();
    if (N == 0) return;

    const double effective_count = (pre_resample_neff > 0.0) ? pre_resample_neff : n_eff_;
    const double ess_ratio = std::clamp(effective_count / static_cast<double>(N), 0.0, 1.0);
    const double degeneracy_factor = 1.0 - ess_ratio; // ESS càng thấp, độ suy thoái càng cao -> tăng jitter
    const double terrain_observability = std::clamp(1.0 - 10.0 * terrain_slope_norm, 0.2, 1.0);

    constexpr double kBaseRougheningStd = 0.4; // [m]
    const double sigma_rough = kBaseRougheningStd * (1.0 + 2.0 * degeneracy_factor) * terrain_observability;

    std::normal_distribution<double> jitter(0.0, sigma_rough);
    for (auto& p : particles_) {
        if (p.is_alive) {
            const double new_e = p.easting_m + jitter(rng_);
            const double new_n = p.northing_m + jitter(rng_);
            if (dem == nullptr || dem->isInside(new_e, new_n)) {
                p.easting_m = new_e;
                p.northing_m = new_n;
            }
        }
    }
}

void ParticleFilterEngine::propagateContinuousMotion(
    double delta_e_m, double delta_n_m, double delta_alt_m,
    double speed_m_s, double dt_s,
    MotionActivity activity) noexcept
{
    if (particles_.empty() || dt_s <= 0.0) return;

    // Stochastic Motion Covariance Q(dt, v, a, act)
    // Tản mạn ngẫu nhiên chuẩn Brownian theo sqrt(dt)
    const double sigma_v = (activity == MotionActivity::kVehicleWheeled) ? 1.2 : 0.6;
    const double alpha_speed = (activity == MotionActivity::kVehicleWheeled) ? 0.08 : 0.03;
    const double horiz_std = std::sqrt(std::max(1e-6, (sigma_v * sigma_v + alpha_speed * speed_m_s) * dt_s));
    const double vert_std = 0.3 * std::sqrt(dt_s);

    std::normal_distribution<double> dist_e(delta_e_m, horiz_std);
    std::normal_distribution<double> dist_n(delta_n_m, horiz_std);
    std::normal_distribution<double> dist_alt(delta_alt_m, vert_std);

    for (auto& p : particles_) {
        if (p.is_alive) {
            p.easting_m += dist_e(rng_);
            p.northing_m += dist_n(rng_);
            p.altitude_m += dist_alt(rng_);
        }
    }
}

void ParticleFilterEngine::setTargetParticleCount(size_t count) noexcept {
    static constexpr size_t kMinParticles = 500;
    static constexpr size_t kMaxParticles = 20000;
    const size_t safe_count = std::clamp(count, kMinParticles, kMaxParticles);
    if (safe_count == particles_.size() || particles_.empty()) {
        return;
    }

    // (1) Chuẩn hóa trọng số cục bộ (caller có thể gọi giữa chu kỳ update)
    double w_total = 0.0;
    for (const auto& p : particles_) {
        w_total += p.weight;
    }
    if (!(w_total > 0.0) || !std::isfinite(w_total)) {
        w_total = static_cast<double>(particles_.size());
    }

    // (2) Systematic resampling N -> safe_count theo trọng số hậu nghiệm:
    //     bảo toàn hình thái phân phối thay vì nhân bản/truncate mù quáng.
    resample_buffer_.resize(safe_count);
    const double r0 = std::uniform_real_distribution<double>(0.0, 1.0)(rng_)
                      / static_cast<double>(safe_count);
    const double inv_w_total = 1.0 / w_total;
    double cum = particles_.front().weight * inv_w_total;
    size_t src = 0;
    for (size_t m = 0; m < safe_count; ++m) {
        const double target = r0 + static_cast<double>(m) / static_cast<double>(safe_count);
        while (cum < target && src + 1 < particles_.size()) {
            ++src;
            cum += particles_[src].weight * inv_w_total;
        }
        resample_buffer_[m] = particles_[src];
    }

    // (3) GROW: jitter Gauss nhẹ phá các bản sao trùng, chống suy biến point-mass
    //     (nếu không, N_eff = N khiến resample/roughening không bao giờ chạy lại).
    if (safe_count > particles_.size()) {
        const double grow_ratio = static_cast<double>(safe_count)
                                  / static_cast<double>(particles_.size());
        const double sigma = std::min(3.0, std::max(0.5, 0.25 * std::sqrt(grow_ratio)));
        std::normal_distribution<double> jitter(0.0, 1.0);
        for (auto& p : resample_buffer_) {
            p.easting_m += jitter(rng_) * sigma;
            p.northing_m += jitter(rng_) * sigma;
            p.altitude_m += jitter(rng_) * sigma * 0.5;
        }
    }

    // (4) Hoán đổi và đặt lại trọng số đều (trạng thái chuẩn hậu-resample).
    //     LƯU Ý: resize() cấp phát heap — hàm chỉ chạy theo SỰ KIỆN PIN, không nằm
    //     trong vòng 50 Hz (Rule.md zero-alloc chỉ áp dụng hot-path).
    particles_.swap(resample_buffer_);
    config_.num_particles = safe_count;

    const double uniform_w = 1.0 / static_cast<double>(safe_count);
    const double uniform_ln = std::log(uniform_w);
    for (auto& p : particles_) {
        p.weight = uniform_w;
        p.log_weight = uniform_ln;
    }
    n_eff_ = static_cast<double>(safe_count);
}

void ParticleFilterEngine::resample() noexcept {
    const size_t N = particles_.size();
    if (N == 0) return;

    // Systematic Resampling O(N) deterministic
    double u0 = std::uniform_real_distribution<double>(0.0, 1.0 / static_cast<double>(N))(rng_);
    double c = particles_[0].weight;
    size_t i = 0;

    for (size_t m = 0; m < N; ++m) {
        const double u = u0 + static_cast<double>(m) / static_cast<double>(N);
        while (u > c && i < N - 1) {
            i++;
            c += particles_[i].weight;
        }
        resample_buffer_[m] = particles_[i];
    }

    const double uniform_weight = 1.0 / static_cast<double>(N);
    const double uniform_log_w = -std::log(static_cast<double>(N));

    for (size_t m = 0; m < N; ++m) {
        particles_[m] = resample_buffer_[m];
        particles_[m].weight = uniform_weight;
        particles_[m].log_weight = uniform_log_w;
        particles_[m].is_alive = true;
    }

    n_eff_ = static_cast<double>(N);
}

EastingNorthingAlt ParticleFilterEngine::getWeightedMeanPosition() const noexcept {
    double sum_w = 0.0;
    double sum_e = 0.0;
    double sum_n = 0.0;
    double sum_alt = 0.0;

    for (const auto& p : particles_) {
        if (!p.is_alive) continue;
        sum_w += p.weight;
        sum_e += p.easting_m * p.weight;
        sum_n += p.northing_m * p.weight;
        sum_alt += p.altitude_m * p.weight;
    }

    if (sum_w <= 1e-12) {
        return {0.0, 0.0, 0.0};
    }

    return {
        .easting_m = sum_e / sum_w,
        .northing_m = sum_n / sum_w,
        .altitude_m = sum_alt / sum_w
    };
}

std::array<double, 4> ParticleFilterEngine::getWeightedCovariance2D() const noexcept {
    const auto mean = getWeightedMeanPosition();
    double sum_w = 0.0;
    double var_e = 0.0;
    double var_n = 0.0;
    double cov_en = 0.0;

    for (const auto& p : particles_) {
        if (!p.is_alive) continue;
        const double de = p.easting_m - mean.easting_m;
        const double dn = p.northing_m - mean.northing_m;
        sum_w += p.weight;
        var_e += de * de * p.weight;
        var_n += dn * dn * p.weight;
        cov_en += de * dn * p.weight;
    }

    if (sum_w <= 1e-12) {
        return {100.0, 0.0, 0.0, 100.0};
    }

    var_e /= sum_w;
    var_n /= sum_w;
    cov_en /= sum_w;

    return {var_e, cov_en, cov_en, var_n};
}

NavigationUncertaintyState ParticleFilterEngine::extractUncertaintyState(uint64_t timestamp_ns) const noexcept {
    NavigationUncertaintyState state{};
    state.timestamp_ms = timestamp_ns / 1000000ULL;

    const auto global_mean = getWeightedMeanPosition();
    const auto global_cov = getWeightedCovariance2D();
    const auto global_ellipse = math::computeUncertaintyEllipse(global_cov[0], global_cov[3], global_cov[1], 5.9915);

    // 1. Phân cụm không gian nhanh Zero-Allocation bằng StaticSpatialHash (Single-Thread Confined member)
    const double cell_sz = config_.grid_cluster_cell_size_m;
    spatial_hash_.clear();

    for (size_t i = 0; i < particles_.size(); ++i) {
        const auto& p = particles_[i];
        if (!p.is_alive || p.weight <= 0.0) continue;

        const int64_t gx = static_cast<int64_t>(std::floor(p.easting_m / cell_sz));
        const int64_t gy = static_cast<int64_t>(std::floor(p.northing_m / cell_sz));

        spatial_hash_.insertOrAdd(gx, gy, p.weight, p.easting_m, p.northing_m, p.altitude_m);
    }

    // 2. Thu thập và sắp xếp các ứng viên cụm từ bảng băm tĩnh
    std::array<math::SpatialCellData, 4> top_clusters{};
    size_t num_found = 0;

    for (const auto& cell : spatial_hash_.getTable()) {
        if (!cell.occupied || cell.weight_sum < config_.cluster_weight_threshold) continue;

        if (num_found < 4) {
            top_clusters[num_found++] = cell;
        } else {
            // Thay thế phần tử nhỏ nhất nếu weight_sum lớn hơn
            size_t min_idx = 0;
            for (size_t k = 1; k < 4; ++k) {
                if (top_clusters[k].weight_sum < top_clusters[min_idx].weight_sum) {
                    min_idx = k;
                }
            }
            if (cell.weight_sum > top_clusters[min_idx].weight_sum) {
                top_clusters[min_idx] = cell;
            }
        }
    }

    // Sắp xếp giảm dần theo trọng số
    std::sort(top_clusters.begin(), top_clusters.begin() + num_found, [](const auto& a, const auto& b) {
        return a.weight_sum > b.weight_sum;
    });

    // [B1] Gộp các ô KỀ nhau (8-neighborhood) thành MỘT giả thuyết — hết hiện tượng
    // một đám mây nằm đè lên biên ô bị tách làm 2 cụm giả thuyết ảo.
    // Union-find trên tối đa 4 ô — zero allocation.
    size_t group_root[4] = {0, 1, 2, 3};
    auto find_root = [&group_root](size_t g) noexcept {
        while (group_root[g] != g) {
            g = group_root[g];
        }
        return g;
    };
    for (size_t a = 0; a < num_found; ++a) {
        for (size_t b = a + 1; b < num_found; ++b) {
            const auto& ca = top_clusters[a];
            const auto& cb = top_clusters[b];
            if (std::abs(ca.gx - cb.gx) <= 1 && std::abs(ca.gy - cb.gy) <= 1) {
                const size_t ra = find_root(a);
                const size_t rb = find_root(b);
                if (ra != rb) {
                    group_root[rb] = ra;
                }
            }
        }
    }

    // Gom các ô theo root nhóm (tối đa 4 nhóm)
    size_t group_ids[4] = {0, 1, 2, 3};
    size_t num_groups = 0;
    for (size_t k = 0; k < num_found; ++k) {
        const size_t root = find_root(k);
        bool exists = false;
        for (size_t g = 0; g < num_groups; ++g) {
            if (group_ids[g] == root) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            group_ids[num_groups++] = root;
        }
    }

    const size_t num_clusters = (num_groups == 0) ? 1 : num_groups;
    state.active_clusters_count = static_cast<uint8_t>(num_clusters);

    if (num_groups == 0) {
        state.clusters[0].cluster_id = 1;
        state.clusters[0].weight = 1.0;
        state.clusters[0].center_easting_m = global_mean.easting_m;
        state.clusters[0].center_northing_m = global_mean.northing_m;
        state.clusters[0].center_altitude_m = global_mean.altitude_m;
        state.clusters[0].ce95_semi_major_m = global_ellipse.semi_major_m;
        state.clusters[0].ce95_semi_minor_m = global_ellipse.semi_minor_m;
        state.clusters[0].ellipse_heading_rad = global_ellipse.orientation_rad;
        state.clusters[0].area_95_m2 = global_ellipse.area_m2;
    } else {
        // [B1] Pass 2: tích lũy moment bậc hai có trọng số TỪNG HẠT theo nhóm —
        // hiệp phương sai THẬT thay vì (cell/2)² cố định 2500 m² khiến mọi cụm
        // báo CE95 ≈ 122 m bất kể phân tán thật.
        struct GroupAccum {
            double w{0.0};
            double se{0.0};
            double sn{0.0};
            double salt{0.0};
            double see{0.0};
            double snn{0.0};
            double sen{0.0};
        };
        GroupAccum acc[4]{};

        for (const auto& p : particles_) {
            if (!p.is_alive || p.weight <= 0.0) continue;
            const int64_t pgx = static_cast<int64_t>(std::floor(p.easting_m / cell_sz));
            const int64_t pgy = static_cast<int64_t>(std::floor(p.northing_m / cell_sz));
            for (size_t g = 0; g < num_groups; ++g) {
                bool member = false;
                for (size_t k = 0; k < num_found; ++k) {
                    if (find_root(k) == group_ids[g] &&
                        top_clusters[k].gx == pgx && top_clusters[k].gy == pgy) {
                        member = true;
                        break;
                    }
                }
                if (member) {
                    auto& a = acc[g];
                    a.w += p.weight;
                    a.se += p.weight * p.easting_m;
                    a.sn += p.weight * p.northing_m;
                    a.salt += p.weight * p.altitude_m;
                    a.see += p.weight * p.easting_m * p.easting_m;
                    a.snn += p.weight * p.northing_m * p.northing_m;
                    a.sen += p.weight * p.easting_m * p.northing_m;
                    break;
                }
            }
        }

        // Sắp nhóm giảm dần theo trọng số (insertion sort trên ≤ 4 phần tử)
        for (size_t i = 1; i < num_groups; ++i) {
            GroupAccum tmp = acc[i];
            size_t j = i;
            while (j > 0 && acc[j - 1].w < tmp.w) {
                acc[j] = acc[j - 1];
                --j;
            }
            acc[j] = tmp;
        }

        double total_w = 0.0;
        for (size_t g = 0; g < num_groups; ++g) {
            total_w += acc[g].w;
        }

        size_t filled = 0;
        for (size_t k = 0; k < num_groups; ++k) {
            const auto& a = acc[k];
            if (a.w <= 1e-12) continue;
            const double mean_e = a.se / a.w;
            const double mean_n = a.sn / a.w;
            const double mean_alt = a.salt / a.w;

            // Phương sai có trọng số (biased 1/Σw — sai lệch ~0.05% với N = 2000)
            double var_e = a.see / a.w - mean_e * mean_e;
            double var_n = a.snn / a.w - mean_n * mean_n;
            double cov_en = a.sen / a.w - mean_e * mean_n;
            // Sàn suy biến: ellipse tối thiểu 0.5 m std; kẹp |cov| ≤ sqrt(var_e·var_n)
            var_e = std::max(var_e, 0.25);
            var_n = std::max(var_n, 0.25);
            cov_en = std::clamp(cov_en, -std::sqrt(var_e * var_n), std::sqrt(var_e * var_n));

            const auto ell = math::computeUncertaintyEllipse(var_e, var_n, cov_en, 5.9915);

            state.clusters[filled].cluster_id = static_cast<uint8_t>(filled + 1);
            state.clusters[filled].weight = (total_w > 0.0) ? (a.w / total_w) : (1.0 / static_cast<double>(num_groups));
            state.clusters[filled].center_easting_m = mean_e;
            state.clusters[filled].center_northing_m = mean_n;
            state.clusters[filled].center_altitude_m = mean_alt;
            state.clusters[filled].ce95_semi_major_m = ell.semi_major_m;
            state.clusters[filled].ce95_semi_minor_m = ell.semi_minor_m;
            state.clusters[filled].ellipse_heading_rad = ell.orientation_rad;
            state.clusters[filled].area_95_m2 = ell.area_m2;
            ++filled;
        }

        if (filled == 0) {
            // Toàn bộ nhóm rỗng (không hạt sống) — fallback toàn cục
            state.clusters[0].cluster_id = 1;
            state.clusters[0].weight = 1.0;
            state.clusters[0].center_easting_m = global_mean.easting_m;
            state.clusters[0].center_northing_m = global_mean.northing_m;
            state.clusters[0].center_altitude_m = global_mean.altitude_m;
            state.clusters[0].ce95_semi_major_m = global_ellipse.semi_major_m;
            state.clusters[0].ce95_semi_minor_m = global_ellipse.semi_minor_m;
            state.clusters[0].ellipse_heading_rad = global_ellipse.orientation_rad;
            state.clusters[0].area_95_m2 = global_ellipse.area_m2;
        } else {
            state.active_clusters_count = static_cast<uint8_t>(filled);
        }
    }

    const double area_penalty = std::clamp(1.0 - (global_ellipse.area_m2 / 50000.0), 0.0, 1.0);
    const double neff_ratio = std::clamp(n_eff_ / static_cast<double>(config_.num_particles), 0.0, 1.0);
    state.global_navigation_score = std::clamp(0.5 * area_penalty + 0.5 * neff_ratio, 0.05, 0.99);
    state.calibrated_containment_probability = state.global_navigation_score;

    return state;
}

} // namespace survival::estimators
