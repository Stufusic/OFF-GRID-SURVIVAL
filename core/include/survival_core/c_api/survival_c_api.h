#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* survival_handle_t;

/**
 * @brief Khởi tạo đối tượng động cơ định vị sinh tồn Off-Grid Survival Engine
 */
survival_handle_t survival_engine_create(void);

/**
 * @brief Hủy đối tượng và giải phóng toàn bộ tài nguyên
 */
void survival_engine_destroy(survival_handle_t handle);

/**
 * @brief Nạp gói bản đồ DEM offline từ tệp nhị phân .sdem
 */
int survival_engine_load_offline_dem(survival_handle_t handle, const char* filepath);

/**
 * [B3] Cài từ sai (magnetic declination) cho từ kế [độ], Đông-dương.
 * Gọi sau khi biết vị trí vùng hoạt động (bản đồ/lịch thiên văn).
 */
void survival_engine_set_declination(survival_handle_t handle, double declination_deg);

/**
 * @brief Đẩy mẫu định vị GPS (1 Hz) vào hệ thống
 */
void survival_engine_push_gps_fix(
    survival_handle_t handle,
    double lat_rad,
    double lon_rad,
    double alt_m,
    double std_m,
    double hdop,
    uint64_t timestamp_ns
);

/**
 * @brief Thông báo ngắt / tắt GPS, kích hoạt chuyển giao mượt mà sang Off-Grid
 */
void survival_engine_push_gps_cutoff(survival_handle_t handle, uint64_t timestamp_ns);

/**
 * @brief Đẩy 1 khung dữ liệu cảm biến đa tốc độ SensorFrame (100 Hz IMU, 20 Hz Mag, 10 Hz Baro)
 * @return 1 nếu có cập nhật trạng thái mới, 0 nếu không
 */
int survival_engine_push_sensor_frame(survival_handle_t handle, const void* sensor_frame_ptr);

/**
 * @brief Đọc trạng thái bất định mới nhất (Zero-Lock, Lock-Free Double-Buffer Read) cho UI Thread
 * @return 1 nếu thành công, 0 nếu lỗi
 */
int survival_engine_get_uncertainty_state(survival_handle_t handle, void* out_state_ptr);

/**
 * @brief Lấy hoạt động chuyển động và độ tin cậy được phân loại
 */
int survival_engine_get_motion_activity(survival_handle_t handle, uint8_t* out_activity, float* out_confidence);

/**
 * @brief Lấy vector vận tốc tức thời 3D (ENU) và tốc độ tức thời ở 100 Hz
 */
int survival_engine_get_instantaneous_velocity(
    survival_handle_t handle,
    double* out_vx,
    double* out_vy,
    double* out_vz,
    double* out_speed
);

/**
 * @brief Lấy khung dữ liệu BLE Advertisement 20-byte cứu hộ khẩn cấp
 */
int survival_engine_get_ble_adv_payload(
    survival_handle_t handle,
    uint8_t* out_buffer,
    size_t max_len,
    size_t* out_len
);

/**
 * @brief Lấy chuỗi SMS cứu nạn khẩn cấp
 */
int survival_engine_get_sms_payload(
    survival_handle_t handle,
    char* out_str,
    size_t max_len
);

/**
 * @brief Cập nhật phần trăm pin thiết bị (SoC [0 - 100%])
 */
void survival_engine_update_battery_soc(survival_handle_t handle, double soc_percent);

/**
 * @brief Người dùng chạm màn hình hủy cảnh báo đếm ngược cứu hộ
 */
void survival_engine_cancel_safety_alert(survival_handle_t handle);

/**
 * @brief Lấy báo cáo giám sát toàn vẹn hệ thống mới nhất (Thread-safe snapshot)
 * @param out_is_healthy 1 nếu hệ thống an toàn/khỏe mạnh, 0 nếu phát hiện bất thường
 * @param out_fault_flags Mã lỗi bitmask (FaultCode)
 * @return 1 nếu thành công, 0 nếu lỗi handle
 */
int survival_engine_get_integrity_status(
    survival_handle_t handle,
    int* out_is_healthy,
    uint32_t* out_fault_flags
);

/**
 * @brief Lấy mã trạng thái an toàn: 0: Normal, 1: PreAlert, 2: DistressLocked, 3: PanicCircling
 */
int survival_engine_get_safety_state(survival_handle_t handle);

/**
 * @brief Tăng thời gian mô phỏng thêm delta_ns (phục vụ kiểm thử watchdog độ tươi cảm biến)
 */
void survival_engine_advance_simulated_time(survival_handle_t handle, uint64_t delta_ns);

#ifdef __cplusplus
}
#endif
