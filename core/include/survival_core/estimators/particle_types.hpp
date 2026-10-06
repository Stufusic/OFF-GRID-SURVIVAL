#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include "survival_core/types.hpp"

namespace survival::estimators {

struct Particle {
    double easting_m{0.0};
    double northing_m{0.0};
    double altitude_m{1000.0};
    double log_weight{0.0};
    double weight{1.0};
    bool is_alive{true};
};

struct ParticleFilterConfig {
    size_t num_particles{2000};                // Số lượng hạt (N = 2.000 đến 5.000)
    double init_pos_std_m{15.0};               // Độ lệch chuẩn bán kính khởi tạo ban đầu [m]
    
    // Trọng số Evidence Tempering Log-Likelihood Fusion
    double lambda_elev{1.0};                   // Trọng số cao độ
    double lambda_slope{0.8};                  // Trọng số độ dốc
    double lambda_aspect{0.6};                 // Trọng số hướng dốc
    double lambda_trav{1.0};                   // Trọng số khả vi hành
    double lambda_sun{0.5};                    // Trọng số quan sát thiên văn Mặt Trời

    // Sai số chuẩn đo lường (Measurement Standard Deviations)
    double sigma_elev_m{3.5};                  // Sai số áp kế vs DEM [m]
    double sigma_slope_rad{0.08};              // Sai số độ dốc PDR vs DEM (~4.5 độ)
    double sigma_aspect_rad{0.35};             // Sai số hướng dốc (~20 độ)
    double sigma_sun_rad{0.08};                // Sai số quan sát Mặt Trời (~4.5 độ)

    // Xử lý suy biến địa hình phẳng (Flat Terrain Degeneracy Protection)
    double aspect_s_min{0.05};                 // Ngưỡng Gradient tối thiểu để phạt Aspect (slope >= ~3 độ)
    double aspect_s0{0.20};                    // Hệ số tỷ lệ hàm tanh(grad / S0)

    // Ngưỡng vách đá không thể đi qua (Traversability Gate)
    double max_traversable_slope_rad{0.96};    // 55 độ = 0.96 rad

    // Phân cụm không gian nhanh (Grid-Based Density Peak Clustering)
    double grid_cluster_cell_size_m{100.0};    // Kích thước ô lưới phân cụm: 100m x 100m
    double cluster_weight_threshold{0.10};     // Ngưỡng mật độ trọng số để thành lập cụm giả thuyết
    double neff_resample_ratio{0.50};          // Tái lấy mẫu khi N_eff < N * 0.50
};

} // namespace survival::estimators
