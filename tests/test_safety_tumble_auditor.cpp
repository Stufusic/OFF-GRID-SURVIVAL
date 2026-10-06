#include <gtest/gtest.h>
#include "survival_core/safety/safety_auditor.hpp"
#include <cmath>

using namespace survival;
using namespace survival::safety;

TEST(SafetyAuditorTest, DeviceDropWithoutTumbleDoesNotTriggerEmergencyAlert) {
    SafetyAuditor auditor;

    // Rơi điện thoại: Tốc độ góc thấp (0.5 rad/s) nhưng gia tốc va đập cao 4.0G tại t = 1.0s
    Vector3f accel_impact{0.0F, 0.0F, 4.0F * 9.81F};
    Vector3f gyro_low{0.1F, 0.2F, 0.3F}; // norm = 0.37 < 3.0 rad/s

    auditor.processMotionSample(accel_impact, gyro_low, 1000000000ULL);

    // Không được kích hoạt PRE_ALERT cứu nạn
    EXPECT_EQ(auditor.getState(), SafetyState::kNormal);
    EXPECT_TRUE(auditor.isDeviceDropOnly());
    EXPECT_FALSE(auditor.isDistressActive());
}

TEST(SafetyAuditorTest, HumanFallWithTumbleTriggersPreAlertAndCountdown) {
    SafetyAuditor auditor;

    // 1. Chuỗi xoay người mất kiểm soát trước va chạm (t = 0.8s, gyro = 3.5 rad/s)
    Vector3f accel_tumble{0.0F, 0.0F, 9.81F};
    Vector3f gyro_high{2.0F, 2.5F, 1.5F}; // norm = sqrt(4 + 6.25 + 2.25) = 3.53 > 3.0 rad/s
    auditor.processMotionSample(accel_tumble, gyro_high, 800000000ULL);

    // 2. Va đập mạnh xuống đất (t = 1.0s, delta = 200ms < 300ms, accel = 4.5G)
    Vector3f accel_impact{0.0F, 0.0F, 4.5F * 9.81F};
    Vector3f gyro_impact{0.5F, 0.5F, 0.5F};
    auditor.processMotionSample(accel_impact, gyro_impact, 1000000000ULL);

    // Phải chuyển sang PRE_ALERT đếm ngược 45s
    EXPECT_EQ(auditor.getState(), SafetyState::kPreAlert);
    EXPECT_FALSE(auditor.isDeviceDropOnly());
    EXPECT_GT(auditor.getPreAlertRemainingSeconds(), 40.0);
    EXPECT_LE(auditor.getPreAlertRemainingSeconds(), 45.0);

    // 3. Người dùng chạm màn hình hủy cảnh báo
    auditor.cancelAlert();
    EXPECT_EQ(auditor.getState(), SafetyState::kNormal);
}

TEST(SafetyAuditorTest, ImmobilityCountdownLocksDistressState) {
    SafetyAuditor auditor;

    // 1. Kích hoạt ngã
    auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{3.2F, 0.0F, 0.0F}, 1000000000ULL);
    auditor.processMotionSample(Vector3f{0.0F, 0.0F, 4.5F * 9.81F}, Vector3f{0.0F, 0.0F, 0.0F}, 1100000000ULL);
    ASSERT_EQ(auditor.getState(), SafetyState::kPreAlert);

    // 2. Trôi qua 46 giây không có tương tác người dùng (t = 47.1s)
    auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{0.0F, 0.0F, 0.0F}, 47100000000ULL);

    // Phải tự động khóa cứu nạn khẩn cấp kDistressLocked
    EXPECT_EQ(auditor.getState(), SafetyState::kDistressLocked);
    EXPECT_TRUE(auditor.isDistressActive());
}

TEST(SafetyAuditorTest, PanicCirclingDetection) {
    SafetyAuditor auditor;

    // Đi 600 bước, tổng quãng đường 450m nhưng chỉ dịch chuyển thực tế 20m (tỷ lệ 20/450 = 0.044 < 0.10)
    auditor.updateOdometryDisplacement(20.0, 450.0, 600);

    EXPECT_TRUE(auditor.isPanicCirclingWarning());
    EXPECT_EQ(auditor.getState(), SafetyState::kPanicCirclingWarning);
}


// [M8a] Roc may thang dung: impact + spin cung mot mau khong duoc tinh la "tumble
// truoc va cham" (truoc fix: mau impact tu no roi vao cua so 300ms -> false PreAlert)
TEST(SafetyAuditorTest, DeviceDropWithImpactSpinIsNotHumanFall) {
    SafetyAuditor auditor{};

    // 1 giay dung yen
    for (int i = 0; i < 100; ++i) {
        auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{0.0F, 0.0F, 0.0F},
                                    static_cast<uint64_t>(i) * 10000000ULL);
    }
    // Roi tu do 300ms (accel ~0, khong xoay)
    for (int i = 100; i < 130; ++i) {
        auditor.processMotionSample(Vector3f{0.0F, 0.0F, 0.0F}, Vector3f{0.0F, 0.0F, 0.0F},
                                    static_cast<uint64_t>(i) * 10000000ULL);
    }
    // Impact 6g KEM xoay 4 rad/s CUNG MOT MAU (may va dat roi vat nhay)
    auditor.processMotionSample(Vector3f{0.0F, 0.0F, 6.0F * 9.81F},
                                Vector3f{4.0F, 0.0F, 0.0F},
                                130ULL * 10000000ULL);
    // Giai toc do sau impact van xoay giam dan
    for (int i = 131; i < 200; ++i) {
        const float g = 4.0F * (1.0F - static_cast<float>(i - 131) / 70.0F);
        auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{g, 0.0F, 0.0F},
                                    static_cast<uint64_t>(i) * 10000000ULL);
    }

    EXPECT_EQ(auditor.getState(), SafetyState::kNormal);  // truoc fix: kPreAlert
    EXPECT_TRUE(auditor.isDeviceDropOnly());
}

// [M8b] Escalation bat dong hoat dong doc lap voi countdown (config countdown 300s)
TEST(SafetyAuditorTest, ImmobilityEscalationFiresBeforeCountdown) {
    SafetyConfig cfg{};
    cfg.pre_alert_duration_ns = 300000000000ULL;  // 300s — de countdown khong che khuat
    SafetyAuditor auditor{cfg};

    // Nga co tumble: xoay 4 rad/s trong 200ms truoc impact 6g
    for (int i = 0; i < 100; ++i) {
        auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{0.0F, 0.0F, 0.0F},
                                    static_cast<uint64_t>(i) * 10000000ULL);
    }
    for (int i = 100; i < 120; ++i) {
        auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{4.0F, 0.0F, 0.0F},
                                    static_cast<uint64_t>(i) * 10000000ULL);
    }
    auditor.processMotionSample(Vector3f{0.0F, 0.0F, 6.0F * 9.81F}, Vector3f{4.0F, 0.0F, 0.0F},
                                120ULL * 10000000ULL);  // t = 1.2s
    EXPECT_EQ(auditor.getState(), SafetyState::kPreAlert);

    // Nan nhan cu dong tai t = 60s (30 giay), roi nam yen
    auditor.processMotionSample(Vector3f{0.0F, 0.0F, 12.0F}, Vector3f{1.0F, 0.0F, 0.0F},
                                6000ULL * 10000000ULL);  // t = 60s: motion
    // Nam yen lien tuc sau do (buoc 0.1s den t = 250s)
    for (int i = 61; i <= 2500; ++i) {
        auditor.processMotionSample(Vector3f{0.0F, 0.0F, 9.81F}, Vector3f{0.0F, 0.0F, 0.0F},
                                    static_cast<uint64_t>(i) * 100000000ULL);
    }
    // Bat dong 180s ke tu lan cu dong cuoi (60s) -> khoa tai 60+180 = 240s
    // Truoc fix: predicate chi tich luc khi last_motion <= impact+5s (60s > 6.2s -> khong bao gio)
    EXPECT_EQ(auditor.getState(), SafetyState::kDistressLocked);
}
