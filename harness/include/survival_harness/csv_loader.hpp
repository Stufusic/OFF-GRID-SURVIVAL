#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include <optional>
#include "survival_harness/synthetic_generator.hpp"
#include "survival_core/data_contracts.hpp"

namespace survival::harness {

enum class TimestampUnit : uint8_t {
    kNanoseconds = 0,
    kMicroseconds = 1,
    kMilliseconds = 2,
    kSeconds = 3
};

enum class AngularVelocityUnit : uint8_t {
    kRadPerSec = 0,
    kDegPerSec = 1
};

enum class AccelerationUnit : uint8_t {
    kMetersPerSec2 = 0,
    kGForce = 1 // 1g = 9.80665 m/s^2
};

struct CsvLoaderConfig {
    TimestampUnit timestamp_unit{TimestampUnit::kNanoseconds};
    AngularVelocityUnit gyro_unit{AngularVelocityUnit::kRadPerSec};
    AccelerationUnit accel_unit{AccelerationUnit::kMetersPerSec2};
    
    // Nếu file không có cột timestamp, dùng dt mặc định (giả định 100 Hz = 0.01s)
    double default_dt_s{0.01};
    
    // Phân cách cột (mặc định ',')
    char delimiter{','};
    
    // Tự động bỏ qua dòng đầu nếu là header
    bool auto_detect_header{true};
};

class CsvLoader {
public:
    /**
     * @brief Đọc dataset từ file CSV trên ổ đĩa
     * @param filepath Đường dẫn file CSV
     * @param config Cấu hình đơn vị và định dạng
     * @return SyntheticDataset chứa toàn bộ SensorFrame và GroundTruth (nếu có)
     */
    [[nodiscard]] static std::optional<SyntheticDataset> loadFromFile(
        const std::string& filepath,
        const CsvLoaderConfig& config = {}
    ) noexcept;

    /**
     * @brief Đọc dataset từ chuỗi nội dung CSV trong bộ nhớ
     * @param csv_content Nội dung chuỗi CSV
     * @param config Cấu hình đơn vị và định dạng
     * @return SyntheticDataset chứa toàn bộ SensorFrame và GroundTruth (nếu có)
     */
    [[nodiscard]] static std::optional<SyntheticDataset> loadFromString(
        std::string_view csv_content,
        const CsvLoaderConfig& config = {}
    ) noexcept;

    /**
     * @brief Đọc trực tiếp từ thư mục xuất của ứng dụng Sensor Logger (Android/iOS)
     *        Tự động đồng bộ TotalAcceleration.csv, Gyroscope.csv, Magnetometer.csv, Orientation.csv
     * @param folder_path Đường dẫn thư mục chứa các file CSV
     * @return SyntheticDataset đồng bộ đa cảm biến
     */
    [[nodiscard]] static std::optional<SyntheticDataset> loadSensorLoggerFolder(
        const std::string& folder_path
    ) noexcept;
};

} // namespace survival::harness
