#include <gtest/gtest.h>
#include "survival_core/math/static_spatial_hash.hpp"
#include <chrono>
#include <random>

using namespace survival::math;

TEST(StaticSpatialHashTest, InsertAndRetrieveCorrectness) {
    StaticSpatialHash<512> hash;

    // Chèn 100 điểm phân bố trên các ô lưới khác nhau
    hash.insertOrAdd(10, 20, 0.5, 100.0, 200.0, 1500.0);
    hash.insertOrAdd(10, 20, 0.5, 102.0, 204.0, 1502.0); // Cùng ô (10, 20) -> cộng dồn
    hash.insertOrAdd(15, 25, 1.0, 150.0, 250.0, 1600.0); // Ô mới

    EXPECT_EQ(hash.size(), 2U);

    size_t found_count = 0;
    for (const auto& cell : hash.getTable()) {
        if (cell.occupied) {
            if (cell.gx == 10 && cell.gy == 20) {
                EXPECT_NEAR(cell.weight_sum, 1.0, 1e-6);
                EXPECT_EQ(cell.count, 2U);
                EXPECT_NEAR(cell.sum_e, 100.0 * 0.5 + 102.0 * 0.5, 1e-6);
                found_count++;
            } else if (cell.gx == 15 && cell.gy == 25) {
                EXPECT_NEAR(cell.weight_sum, 1.0, 1e-6);
                EXPECT_EQ(cell.count, 1U);
                found_count++;
            }
        }
    }
    EXPECT_EQ(found_count, 2U);
}

TEST(StaticSpatialHashTest, WorstCaseLatencyUnder1msFor2000Particles) {
    StaticSpatialHash<512> hash;
    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist_pos(-500.0, 500.0);
    std::uniform_real_distribution<double> dist_w(0.0001, 0.01);

    constexpr size_t kNumParticles = 2000;
    struct ParticleMock {
        double e, n, alt, w;
    };
    std::vector<ParticleMock> particles(kNumParticles);
    for (size_t i = 0; i < kNumParticles; ++i) {
        particles[i] = {dist_pos(rng), dist_pos(rng), 1500.0, dist_w(rng)};
    }

    const auto t_start = std::chrono::high_resolution_clock::now();

    hash.clear();
    for (size_t i = 0; i < kNumParticles; ++i) {
        const int64_t gx = static_cast<int64_t>(std::floor(particles[i].e / 20.0));
        const int64_t gy = static_cast<int64_t>(std::floor(particles[i].n / 20.0));
        hash.insertOrAdd(gx, gy, particles[i].w, particles[i].e, particles[i].n, particles[i].alt);
    }

    const auto t_end = std::chrono::high_resolution_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // Thời gian thực thi gom cụm 2000 hạt trong môi trường debug < 5.0 ms, release < 0.2 ms
    EXPECT_LT(elapsed_ms, 5.0);
}
