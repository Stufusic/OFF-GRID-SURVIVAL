#include <gtest/gtest.h>
#include "survival_core/c_api/survival_c_api.h"
#include "survival_core/data_contracts.hpp"
#include "survival_core/storage/local_blackbox.hpp"
#include "survival_core/environment/dem_map.hpp"
#include <cstdio>
#include <cstring>
#include <thread>
#include <atomic>

using namespace survival;

TEST(LockFreeCApiTest, LifecycleAndDoubleBufferRead) {
    // 1. Tạo file DEM tạm
    environment::DemMap dem;
    ASSERT_TRUE(dem.createSyntheticCone(1000.0, 1000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 400.0));
    const std::string dem_file = "test_c_api_cone.sdem";
    ASSERT_TRUE(dem.saveToFile(dem_file));

    // 2. Khởi tạo Engine qua C-API
    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);

    EXPECT_EQ(survival_engine_load_offline_dem(engine, dem_file.c_str()), 1);

    // 3. Đẩy luồng GPS fix
    survival_engine_push_gps_fix(engine, 0.3665, 1.8326, 1200.0, 3.0, 1.0, 1000000000ULL);

    // 4. Đẩy khung cảm biến
    SensorFrame frame{};
    frame.timestamp_ns = 1010000000ULL;
    frame.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    frame.gyro_rad_s = {0.0F, 0.0F, 0.0F};
    frame.pressure_hpa = 1013.25F;
    frame.has_imu = true;
    frame.has_baro = true;

    // Đẩy frame vào engine
    survival_engine_push_sensor_frame(engine, &frame);

    // 5. Đọc trạng thái từ UI Thread (Lock-Free Read)
    NavigationUncertaintyState state{};
    const int read_ok = survival_engine_get_uncertainty_state(engine, &state);
    EXPECT_EQ(read_ok, 1);

    // 6. Lấy BLE Advertisement Payload (24 bytes chuẩn)
    uint8_t ble_buf[64];
    size_t ble_len = 0;
    const int ble_ok = survival_engine_get_ble_adv_payload(engine, ble_buf, sizeof(ble_buf), &ble_len);
    EXPECT_EQ(ble_ok, 1);
    EXPECT_EQ(ble_len, 24U);
    EXPECT_EQ(ble_buf[1], 0xFF); // AD Type Manufacturer Specific

    // 7. Lấy chuỗi SMS cứu nạn
    char sms_buf[256];
    const int sms_ok = survival_engine_get_sms_payload(engine, sms_buf, sizeof(sms_buf));
    EXPECT_EQ(sms_ok, 1);
    EXPECT_TRUE(std::strstr(sms_buf, "SOS") != nullptr);

    // 8. Hủy Engine
    survival_engine_destroy(engine);
    std::remove(dem_file.c_str());
}

TEST(LockFreeCApiTest, HighConcurrencyLockFreeNoDataRace) {
    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);

    std::atomic<bool> stop_flag{false};
    std::atomic<size_t> sensor_pushes{0};
    std::atomic<size_t> ui_reads{0};

    // Luồng 1: Sensor Thread chạy ở 100 Hz (đẩy 500 mẫu)
    std::thread sensor_thread([&]() {
        SensorFrame f{};
        f.accel_m_s2 = {0.0F, 0.0F, 9.81F};
        f.has_imu = true;
        for (uint64_t i = 1; i <= 300; ++i) {
            f.timestamp_ns = i * 10000000ULL;
            survival_engine_push_sensor_frame(engine, &f);
            sensor_pushes++;
            std::this_thread::yield();
        }
        stop_flag.store(true);
    });

    // Luồng 2: UI Thread liên tục đọc không khoá (Lock-Free Double Buffer)
    std::thread ui_thread([&]() {
        NavigationUncertaintyState state{};
        while (!stop_flag.load()) {
            survival_engine_get_uncertainty_state(engine, &state);
            ui_reads++;
            std::this_thread::yield();
        }
    });

    sensor_thread.join();
    ui_thread.join();

    EXPECT_GE(sensor_pushes.load(), 300U);
    EXPECT_GE(ui_reads.load(), 50U);

    survival_engine_destroy(engine);
}

// ============================================================================
// [H5] Đồng thời nhiều reader + 1 writer qua C API (triple buffer N-readers)
// ============================================================================
#include <cmath>
#include <vector>

namespace {
// Dựng engine có DEM để writer publish state liên tục.
// Mỗi test dùng tên file RIÊNG: ctest -j chạy các test song song trong cùng thư mục.
survival_handle_t makeStressEngine(const char* dem_file) {
    environment::DemMap dem;
    if (!dem.createSyntheticCone(1000.0, 1000.0, 10.0, 500.0, 500.0, 2000.0, 1000.0, 400.0)) {
        return nullptr;
    }
    if (!dem.saveToFile(dem_file)) {
        return nullptr;
    }
    survival_handle_t engine = survival_engine_create();
    if (engine == nullptr) {
        return nullptr;
    }
    if (survival_engine_load_offline_dem(engine, dem_file) != 1) {
        survival_engine_destroy(engine);
        return nullptr;
    }
    return engine;
}
} // namespace

TEST(CApiConcurrency, TwoReadersOneWriterStress) {
    const char* dem_file = "test_c_api_stress_a.sdem";
    survival_handle_t engine = makeStressEngine(dem_file);
    ASSERT_NE(engine, nullptr);

    std::atomic<bool> stop{false};
    std::atomic<int> writer_frames{0};
    std::atomic<int> reader1_reads{0};
    std::atomic<int> reader2_reads{0};

    // Writer: đẩy ~1500 frame liên tục (publish state + BLE/SMS payload)
    std::thread writer([engine, &stop, &writer_frames]() {
        for (int i = 0; i < 1500 && !stop.load(); ++i) {
            SensorFrame frame{};
            frame.timestamp_ns = static_cast<uint64_t>(1000000000ULL + i * 10000000ULL);
            frame.accel_m_s2 = {0.0F, 0.0F, 9.81F};
            frame.gyro_rad_s = {0.0F, 0.0F, 0.0F};
            frame.pressure_hpa = 1013.25F;
            frame.has_imu = true;
            frame.has_baro = true;
            // push trả 1 khi pipeline emit state (gate >= 1 Hz), 0 khi chỉ tích hợp ngầm
            // — cả hai đều là push hợp lệ cho mục đích stress triple-buffer
            (void)survival_engine_push_sensor_frame(engine, &frame);
            writer_frames.fetch_add(1);
        }
        stop.store(true);
    });

    // Reader 1: UI thread đọc trạng thái — mọi state phải nguyên vẹn (không torn read)
    std::thread reader1([engine, &stop, &reader1_reads]() {
        NavigationUncertaintyState state{};
        while (!stop.load()) {
            if (survival_engine_get_uncertainty_state(engine, &state) == 1) {
                for (int c = 0; c < 4; ++c) {
                    const auto& cl = state.clusters[static_cast<size_t>(c)];
                    EXPECT_FALSE(std::isnan(cl.center_easting_m));
                    EXPECT_FALSE(std::isnan(cl.center_northing_m));
                    EXPECT_GE(cl.ce95_semi_major_m, 0.0);
                }
                EXPECT_FALSE(std::isnan(state.clusters[0].center_altitude_m));
                reader1_reads.fetch_add(1);
            }
        }
    });

    // Reader 2: luồng cứu hộ đọc BLE + SMS — payload phải có độ dài/định dạng chuẩn
    std::thread reader2([engine, &stop, &reader2_reads]() {
        uint8_t ble_buf[64] = {};
        char sms_buf[256] = {};
        while (!stop.load()) {
            size_t ble_len = 0;
            if (survival_engine_get_ble_adv_payload(engine, ble_buf, sizeof(ble_buf), &ble_len) == 1) {
                EXPECT_EQ(ble_len, 24U);
                reader2_reads.fetch_add(1);
            }
            if (survival_engine_get_sms_payload(engine, sms_buf, sizeof(sms_buf)) == 1) {
                EXPECT_TRUE(std::strstr(sms_buf, "SOS") != nullptr);
            }
        }
    });

    writer.join();
    reader1.join();
    reader2.join();

    EXPECT_EQ(writer_frames.load(), 1500);  // mọi frame đẩy thành công
    EXPECT_GT(reader1_reads.load(), 100);
    EXPECT_GT(reader2_reads.load(), 100);
    survival_engine_destroy(engine);
    std::remove(dem_file);
}

TEST(CApiConcurrency, IntegrityStatusIsRaceFree) {
    const char* dem_file = "test_c_api_stress_b.sdem";
    survival_handle_t engine = makeStressEngine(dem_file);
    ASSERT_NE(engine, nullptr);

    std::atomic<bool> stop{false};

    // Luồng A: đẩy đồng hồ mô phỏng (tick thread)
    std::thread ticker([engine, &stop]() {
        while (!stop.load()) {
            survival_engine_advance_simulated_time(engine, 1000000ULL);
        }
    });

    // Luồng B: đọc snapshot toàn vẹn liên tục — không được crash, giá trị hợp lệ
    int reads = 0;
    while (reads < 5000) {
        int is_healthy = -1;
        uint32_t fault_flags = 0;
        ASSERT_EQ(survival_engine_get_integrity_status(engine, &is_healthy, &fault_flags), 1);
        EXPECT_TRUE(is_healthy == 0 || is_healthy == 1);
        reads++;
    }

    stop.store(true);
    ticker.join();
    survival_engine_destroy(engine);
    std::remove(dem_file);
}


// [M9] Engine chua co diem neo hop le -> SMS phai bao NO-FIX, cam phat (0,0)
TEST(CApiNoFix, SmsWithoutFixEmitsNoCoordinates) {
    survival_handle_t engine = survival_engine_create();
    ASSERT_NE(engine, nullptr);
    // KHONG load DEM, KHONG push GPS fix -> local frame origin (0,0) + khong LastKnownGoodFix

    // Dam bao co trang thai published (push 1 frame - pipeline tu khoi tao)
    SensorFrame frame{};
    frame.timestamp_ns = 1000000000ULL;
    frame.accel_m_s2 = {0.0F, 0.0F, 9.81F};
    frame.has_imu = true;
    (void)survival_engine_push_sensor_frame(engine, &frame);

    char sms_buf[256] = {};
    const int ok = survival_engine_get_sms_payload(engine, sms_buf, sizeof(sms_buf));
    EXPECT_EQ(ok, 1);

    // Truoc fix: SMS chua LAT:0.000000 LON:0.000000 + link maps Null Island
    EXPECT_TRUE(std::strstr(sms_buf, "NO-FIX") != nullptr);
    EXPECT_TRUE(std::strstr(sms_buf, "LAT:") == nullptr);
    EXPECT_TRUE(std::strstr(sms_buf, "maps.google.com") == nullptr);

    survival_engine_destroy(engine);
}


// [M15] survival_engine_create phai dang ky platform hooks — blackbox co PAL fsync
// va tu-flush hoat dong qua C API thuan (truoc day hooks chi duoc goi trong test)
TEST(CApiPlatformHooks, BlackboxAutoFlushWorksFromEngineLifecycle) {
    const char* dem_file = "test_c_api_blackbox.sdem";
    survival_handle_t engine = makeStressEngine(dem_file);
    ASSERT_NE(engine, nullptr);

    // 12000 frame x 10ms = 120s mo phong -> ~120 trang thai publish -> tu-flush tai 100
    for (int i = 0; i < 12000; ++i) {
        SensorFrame frame{};
        frame.timestamp_ns = static_cast<uint64_t>(1000000000ULL + i * 10000000ULL);
        frame.accel_m_s2 = {0.0F, 0.0F, 9.81F};
        frame.gyro_rad_s = {0.0F, 0.0F, 0.0F};
        frame.pressure_hpa = 1013.25F;
        frame.has_imu = true;
        frame.has_baro = true;
        (void)survival_engine_push_sensor_frame(engine, &frame);
    }
    survival_engine_destroy(engine);

    // Blackbox v2 phai ghi du file va phuc hoi duoc (header + CRC tung record)
    const auto recovered = survival::storage::LocalBlackbox::recoverRecords("survival_blackbox.bin");
    ASSERT_TRUE(recovered.has_value());
    EXPECT_GE(recovered->size(), 100U);
    std::remove("survival_blackbox.bin");
    std::remove(dem_file);
}
