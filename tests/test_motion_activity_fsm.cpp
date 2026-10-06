#include <gtest/gtest.h>
#include "survival_core/kinematics/motion_transition_fsm.hpp"

using namespace survival;
using namespace survival::kinematics;

TEST(MotionActivityFsmTest, TransitionHysteresisAndSmoothing) {
    MotionTransitionFsm fsm{};

    EXPECT_EQ(fsm.getCurrentActivity(), MotionActivity::kStationary);

    // 1. Gửi 1 frame đi bộ với raw_score cao (0.90) -> chưa chuyển ngay vì cần temporal accumulation
    std::array<float, 12> walk_scores{};
    walk_scores.fill(0.0F);
    walk_scores[static_cast<size_t>(MotionActivity::kWalkingFlat)] = 0.90F;

    auto est = fsm.update(walk_scores, 0.01);
    // Nhờ FSM hysteresis và dwell time, không chuyển ngay sau 1 frame đơn lẻ
    EXPECT_EQ(est.current_activity, MotionActivity::kStationary);

    // 2. Duy trì tín hiệu đi bộ trong 1.5 giây (150 mẫu ở 100Hz)
    for (int i = 0; i < 150; ++i) {
        est = fsm.update(walk_scores, 0.01);
    }
    EXPECT_EQ(est.current_activity, MotionActivity::kWalkingFlat);
    EXPECT_GT(est.fused_confidence, 0.70F);

    // 3. Chuyển sang Leo cầu thang kStairsAscent
    std::array<float, 12> stair_scores{};
    stair_scores.fill(0.0F);
    stair_scores[static_cast<size_t>(MotionActivity::kStairsAscent)] = 0.92F;

    for (int i = 0; i < 150; ++i) {
        est = fsm.update(stair_scores, 0.01);
    }
    EXPECT_EQ(est.current_activity, MotionActivity::kStairsAscent);

    // 4. Chuyển sang Chiếu nghỉ kStairLanding
    std::array<float, 12> landing_scores{};
    landing_scores.fill(0.0F);
    landing_scores[static_cast<size_t>(MotionActivity::kStairLanding)] = 0.88F;

    for (int i = 0; i < 150; ++i) {
        est = fsm.update(landing_scores, 0.01);
    }
    EXPECT_EQ(est.current_activity, MotionActivity::kStairLanding);
}

TEST(MotionActivityFsmTest, PreventInvalidDirectGraphJumps) {
    // Không được nhảy trực tiếp từ kStairsAscent sang kVehicleWheeled
    EXPECT_FALSE(MotionTransitionFsm::isValidTransition(MotionActivity::kStairsAscent, MotionActivity::kVehicleWheeled));

    // Không được nhảy trực tiếp từ kRunning sang kVehicleWheeled
    EXPECT_FALSE(MotionTransitionFsm::isValidTransition(MotionActivity::kRunning, MotionActivity::kVehicleWheeled));

    // Hợp lệ: kStairsAscent -> kStairLanding
    EXPECT_TRUE(MotionTransitionFsm::isValidTransition(MotionActivity::kStairsAscent, MotionActivity::kStairLanding));

    // Hợp lệ: kVehicleWheeled -> kStationary
    EXPECT_TRUE(MotionTransitionFsm::isValidTransition(MotionActivity::kVehicleWheeled, MotionActivity::kStationary));
}
