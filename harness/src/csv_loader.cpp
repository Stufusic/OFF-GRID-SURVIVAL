#include "survival_harness/csv_loader.hpp"
#include "survival_core/types.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <charconv>
#include <cctype>
#include <filesystem>
#include <vector>
#include <cmath>

namespace survival::harness {

namespace {

// Hàm chuẩn hóa chuỗi để so sánh header
[[nodiscard]] std::string normalizeHeader(std::string_view sv) {
    std::string result;
    result.reserve(sv.size());
    for (char c : sv) {
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n' && c != '_' && c != '-' && c != '(' && c != ')' && c != '[' && c != ']') {
            result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
    }
    return result;
}

// [M11] Loại bỏ UTF-8 BOM (EF BB BF) — CSV xuất từ Excel luôn có BOM làm hỏng
// việc map cột timestamp (cột đầu tiên không khớp header)
void stripBom(std::string_view& sv) noexcept {
    if (sv.size() >= 3 &&
        static_cast<unsigned char>(sv[0]) == 0xEF &&
        static_cast<unsigned char>(sv[1]) == 0xBB &&
        static_cast<unsigned char>(sv[2]) == 0xBF) {
        sv.remove_prefix(3);
    }
}

[[nodiscard]] std::string_view trim(std::string_view sv) {
    while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t' || sv.front() == '\r' || sv.front() == '\n')) {
        sv.remove_prefix(1);
    }
    while (!sv.empty() && (sv.back() == ' ' || sv.back() == '\t' || sv.back() == '\r' || sv.back() == '\n')) {
        sv.remove_suffix(1);
    }
    return sv;
}

[[nodiscard]] std::vector<std::string_view> splitLine(std::string_view line, char delimiter) {
    std::vector<std::string_view> tokens;
    size_t start = 0;
    while (start < line.size()) {
        size_t end = line.find(delimiter, start);
        if (end == std::string_view::npos) {
            tokens.push_back(trim(line.substr(start)));
            break;
        }
        tokens.push_back(trim(line.substr(start, end - start)));
        start = end + 1;
    }
    return tokens;
}

[[nodiscard]] double parseDouble(std::string_view sv, double default_val = 0.0) {
    if (sv.empty()) return default_val;
    char buffer[64];
    size_t len = std::min(sv.size(), sizeof(buffer) - 1);
    std::copy_n(sv.data(), len, buffer);
    buffer[len] = '\0';
    char* end_ptr = nullptr;
    double val = std::strtod(buffer, &end_ptr);
    if (end_ptr == buffer) return default_val;
    return val;
}

// [M11] Parse nghiêm ngặt: trả nullopt khi rỗng/không parse được/NaN/Inf —
// thay cho hành vi cũ âm thầm trả 0.0 (nhiễu độc lập của các cột khác nướng 0)
[[nodiscard]] std::optional<double> tryParseDouble(std::string_view sv) noexcept {
    if (sv.empty()) return std::nullopt;
    char buffer[64];
    size_t len = std::min(sv.size(), sizeof(buffer) - 1);
    std::copy_n(sv.data(), len, buffer);
    buffer[len] = '\0';
    char* end_ptr = nullptr;
    const double val = std::strtod(buffer, &end_ptr);
    if (end_ptr == buffer) return std::nullopt;
    if (std::isnan(val) || std::isinf(val)) return std::nullopt;
    return val;
}

struct ColumnMapping {
    int timestamp_idx{-1};
    int accel_x_idx{-1};
    int accel_y_idx{-1};
    int accel_z_idx{-1};
    int gyro_x_idx{-1};
    int gyro_y_idx{-1};
    int gyro_z_idx{-1};
    int mag_x_idx{-1};
    int mag_y_idx{-1};
    int mag_z_idx{-1};
    int pressure_idx{-1};
    int gt_easting_idx{-1};
    int gt_northing_idx{-1};
    int gt_alt_idx{-1};
    int gt_heading_idx{-1};
    bool gt_heading_is_deg{false};  // [M11] header "gtheadingdeg" -> nhân kDegToRad
    int generic_x_idx{-1};
    int generic_y_idx{-1};
    int generic_z_idx{-1};
};

[[nodiscard]] ColumnMapping mapColumns(const std::vector<std::string_view>& headers) {
    ColumnMapping mapping;
    for (size_t i = 0; i < headers.size(); ++i) {
        const std::string norm = normalizeHeader(headers[i]);
        int idx = static_cast<int>(i);

        if (norm == "timestamp" || norm == "timestampns" || norm == "timems" || norm == "timens" || norm == "time" || norm == "t" || norm == "times") {
            mapping.timestamp_idx = idx;
        } else if (norm == "accelx" || norm == "accx" || norm == "ax" || norm == "accelerometerx" || norm == "linearaccelerationx") {
            mapping.accel_x_idx = idx;
        } else if (norm == "accely" || norm == "accy" || norm == "ay" || norm == "accelerometery" || norm == "linearaccelerationy") {
            mapping.accel_y_idx = idx;
        } else if (norm == "accelz" || norm == "accz" || norm == "az" || norm == "accelerometerz" || norm == "linearaccelerationz") {
            mapping.accel_z_idx = idx;
        } else if (norm == "gyrox" || norm == "gyrx" || norm == "gx" || norm == "gyroscopex") {
            mapping.gyro_x_idx = idx;
        } else if (norm == "gyroy" || norm == "gyry" || norm == "gy" || norm == "gyroscopey") {
            mapping.gyro_y_idx = idx;
        } else if (norm == "gyroz" || norm == "gyrz" || norm == "gz" || norm == "gyroscopez") {
            mapping.gyro_z_idx = idx;
        } else if (norm == "magx" || norm == "mx" || norm == "magnetometerx") {
            mapping.mag_x_idx = idx;
        } else if (norm == "magy" || norm == "my" || norm == "magnetometery") {
            mapping.mag_y_idx = idx;
        } else if (norm == "magz" || norm == "mz" || norm == "magnetometerz") {
            mapping.mag_z_idx = idx;
        } else if (norm == "pressure" || norm == "baro" || norm == "press" || norm == "p" || norm == "barometer" || norm == "pressurehpa") {
            mapping.pressure_idx = idx;
        } else if (norm == "gteasting" || norm == "gtx" || norm == "truthx" || norm == "easting" || norm == "posx") {
            mapping.gt_easting_idx = idx;
        } else if (norm == "gtnorthing" || norm == "gty" || norm == "truthy" || norm == "northing" || norm == "posy") {
            mapping.gt_northing_idx = idx;
        } else if (norm == "gtalt" || norm == "gtaltitude" || norm == "gtz" || norm == "truthz" || norm == "alt" || norm == "altitude" || norm == "posz") {
            mapping.gt_alt_idx = idx;
        } else if (norm == "gtheading" || norm == "gtheadingdeg" || norm == "gtheadingrad" || norm == "truthheading" || norm == "heading" || norm == "yaw") {
            mapping.gt_heading_idx = idx;
            mapping.gt_heading_is_deg = (norm == "gtheadingdeg");
        } else if (norm == "x") {
            mapping.generic_x_idx = idx;
        } else if (norm == "y") {
            mapping.generic_y_idx = idx;
        } else if (norm == "z") {
            mapping.generic_z_idx = idx;
        }
    }
    return mapping;
}

struct Timestamped3D {
    uint64_t timestamp_ns{0};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

struct TimestampedScalar {
    uint64_t timestamp_ns{0};
    double value{0.0};
};

std::vector<Timestamped3D> read3DSeries(const std::string& filepath) {
    std::vector<Timestamped3D> series;
    std::ifstream file(filepath);
    if (!file.is_open()) return series;

    std::string line;
    bool header_read = false;
    int ts_col = -1, x_col = -1, y_col = -1, z_col = -1;

    while (std::getline(file, line)) {
        std::string_view sv_line = trim(line);
        if (sv_line.empty() || sv_line.front() == '#') continue;

        stripBom(sv_line);  // [M11]
        auto tokens = splitLine(sv_line, ',');
        if (tokens.empty()) continue;

        if (!header_read) {
            header_read = true;
            for (size_t i = 0; i < tokens.size(); ++i) {
                std::string norm = normalizeHeader(tokens[i]);
                int idx = static_cast<int>(i);
                if (norm == "time" || norm == "timestamp" || norm == "timestampns") ts_col = idx;
                else if (norm == "x" || norm == "ax" || norm == "gx" || norm == "mx" || norm == "accelx" || norm == "gyrox" || norm == "magx") x_col = idx;
                else if (norm == "y" || norm == "ay" || norm == "gy" || norm == "my" || norm == "accely" || norm == "gyroy" || norm == "magy") y_col = idx;
                else if (norm == "z" || norm == "az" || norm == "gz" || norm == "mz" || norm == "accelz" || norm == "gyroz" || norm == "magz") z_col = idx;
            }
            continue;
        }

        if (ts_col >= 0 && x_col >= 0 && y_col >= 0 && z_col >= 0) {
            if (ts_col < static_cast<int>(tokens.size()) &&
                x_col < static_cast<int>(tokens.size()) &&
                y_col < static_cast<int>(tokens.size()) &&
                z_col < static_cast<int>(tokens.size())) {
                Timestamped3D item{};
                item.timestamp_ns = static_cast<uint64_t>(parseDouble(tokens[static_cast<size_t>(ts_col)]));
                item.x = static_cast<float>(parseDouble(tokens[static_cast<size_t>(x_col)]));
                item.y = static_cast<float>(parseDouble(tokens[static_cast<size_t>(y_col)]));
                item.z = static_cast<float>(parseDouble(tokens[static_cast<size_t>(z_col)]));
                series.push_back(item);
            }
        }
    }
    return series;
}

std::vector<TimestampedScalar> readScalarSeries(const std::string& filepath, const std::string& col_target) {
    std::vector<TimestampedScalar> series;
    std::ifstream file(filepath);
    if (!file.is_open()) return series;

    std::string line;
    bool header_read = false;
    int ts_col = -1, val_col = -1;

    while (std::getline(file, line)) {
        std::string_view sv_line = trim(line);
        if (sv_line.empty() || sv_line.front() == '#') continue;

        stripBom(sv_line);  // [M11]
        auto tokens = splitLine(sv_line, ',');
        if (tokens.empty()) continue;

        if (!header_read) {
            header_read = true;
            for (size_t i = 0; i < tokens.size(); ++i) {
                std::string norm = normalizeHeader(tokens[i]);
                int idx = static_cast<int>(i);
                if (norm == "time" || norm == "timestamp" || norm == "timestampns") ts_col = idx;
                else if (norm == col_target) val_col = idx;
            }
            continue;
        }

        if (ts_col >= 0 && val_col >= 0 &&
            ts_col < static_cast<int>(tokens.size()) &&
            val_col < static_cast<int>(tokens.size())) {
            TimestampedScalar item{};
            item.timestamp_ns = static_cast<uint64_t>(parseDouble(tokens[static_cast<size_t>(ts_col)]));
            item.value = parseDouble(tokens[static_cast<size_t>(val_col)]);
            series.push_back(item);
        }
    }
    return series;
}

} // anonymous namespace

std::optional<SyntheticDataset> CsvLoader::loadFromString(
    std::string_view csv_content,
    const CsvLoaderConfig& config
) noexcept {
    if (csv_content.empty()) {
        return std::nullopt;
    }

    SyntheticDataset dataset;
    ColumnMapping mapping;
    bool header_processed = false;
    uint64_t simulated_timestamp_ns = 0;
    const uint64_t dt_ns = static_cast<uint64_t>(config.default_dt_s * 1e9);

    size_t line_start = 0;
    size_t row_count = 0;
    size_t row_parse_errors = 0;  // [M11] số mẫu có cột cảm biến parse thất bại
    while (line_start < csv_content.size()) {
        size_t line_end = csv_content.find('\n', line_start);
        std::string_view raw_line = (line_end == std::string_view::npos) 
            ? csv_content.substr(line_start) 
            : csv_content.substr(line_start, line_end - line_start);
        
        line_start = (line_end == std::string_view::npos) ? csv_content.size() : line_end + 1;
        
        std::string_view line = trim(raw_line);
        stripBom(line);  // [M11]
        if (line.empty() || line.front() == '#') {
            continue;
        }

        const auto tokens = splitLine(line, config.delimiter);
        if (tokens.empty()) continue;

        if (!header_processed && config.auto_detect_header) {
            bool has_alpha = false;
            for (const auto& tok : tokens) {
                for (char c : tok) {
                    if (std::isalpha(static_cast<unsigned char>(c))) {
                        has_alpha = true;
                        break;
                    }
                }
                if (has_alpha) break;
            }

            if (has_alpha) {
                mapping = mapColumns(tokens);
                header_processed = true;
                continue;
            }
            header_processed = true;
        }

        // Xử lý mapping generic x, y, z nếu chưa map
        if (mapping.accel_x_idx == -1 && mapping.generic_x_idx != -1) {
            mapping.accel_x_idx = mapping.generic_x_idx;
            mapping.accel_y_idx = mapping.generic_y_idx;
            mapping.accel_z_idx = mapping.generic_z_idx;
        }

        if (mapping.accel_x_idx == -1 && tokens.size() >= 7) {
            mapping.timestamp_idx = 0;
            mapping.accel_x_idx = 1;
            mapping.accel_y_idx = 2;
            mapping.accel_z_idx = 3;
            mapping.gyro_x_idx = 4;
            mapping.gyro_y_idx = 5;
            mapping.gyro_z_idx = 6;
            if (tokens.size() >= 10) {
                mapping.mag_x_idx = 7;
                mapping.mag_y_idx = 8;
                mapping.mag_z_idx = 9;
            }
            if (tokens.size() >= 11) {
                mapping.pressure_idx = 10;
            }
        }

        SensorFrame frame{};

        // 1. Timestamp
        if (mapping.timestamp_idx >= 0 && mapping.timestamp_idx < static_cast<int>(tokens.size())) {
            double raw_ts = parseDouble(tokens[static_cast<size_t>(mapping.timestamp_idx)]);
            switch (config.timestamp_unit) {
                case TimestampUnit::kNanoseconds:
                    frame.timestamp_ns = static_cast<uint64_t>(raw_ts);
                    break;
                case TimestampUnit::kMicroseconds:
                    frame.timestamp_ns = static_cast<uint64_t>(raw_ts * 1e3);
                    break;
                case TimestampUnit::kMilliseconds:
                    frame.timestamp_ns = static_cast<uint64_t>(raw_ts * 1e6);
                    break;
                case TimestampUnit::kSeconds:
                    frame.timestamp_ns = static_cast<uint64_t>(raw_ts * 1e9);
                    break;
            }
        } else {
            frame.timestamp_ns = simulated_timestamp_ns;
            simulated_timestamp_ns += dt_ns;
        }

        // 2. Accel — [M11] đếm lỗi parse thay vì nuốt im lặng thành 0.0
        double accel_scale = (config.accel_unit == AccelerationUnit::kGForce) ? kGravityStandard : 1.0;
        {
            const int accel_cols[3] = {mapping.accel_x_idx, mapping.accel_y_idx, mapping.accel_z_idx};
            for (int c = 0; c < 3; ++c) {
                if (accel_cols[c] >= 0 && accel_cols[c] < static_cast<int>(tokens.size())) {
                    const auto v = tryParseDouble(tokens[static_cast<size_t>(accel_cols[c])]);
                    if (v.has_value()) {
                        frame.accel_m_s2[static_cast<size_t>(c)] = static_cast<float>(v.value() * accel_scale);
                    } else {
                        row_parse_errors++;
                    }
                }
            }
        }

        // 3. Gyro
        double gyro_scale = (config.gyro_unit == AngularVelocityUnit::kDegPerSec) ? kDegToRad : 1.0;
        if (mapping.gyro_x_idx >= 0 && mapping.gyro_x_idx < static_cast<int>(tokens.size())) {
            frame.gyro_rad_s[0] = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.gyro_x_idx)]) * gyro_scale);
        }
        if (mapping.gyro_y_idx >= 0 && mapping.gyro_y_idx < static_cast<int>(tokens.size())) {
            frame.gyro_rad_s[1] = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.gyro_y_idx)]) * gyro_scale);
        }
        if (mapping.gyro_z_idx >= 0 && mapping.gyro_z_idx < static_cast<int>(tokens.size())) {
            frame.gyro_rad_s[2] = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.gyro_z_idx)]) * gyro_scale);
        }

        // 4. Mag
        if (mapping.mag_x_idx >= 0 && mapping.mag_x_idx < static_cast<int>(tokens.size())) {
            frame.mag_uT[0] = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.mag_x_idx)]));
            frame.has_mag = true;
        } else {
            frame.has_mag = false;
        }
        if (mapping.mag_y_idx >= 0 && mapping.mag_y_idx < static_cast<int>(tokens.size())) {
            frame.mag_uT[1] = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.mag_y_idx)]));
        }
        if (mapping.mag_z_idx >= 0 && mapping.mag_z_idx < static_cast<int>(tokens.size())) {
            frame.mag_uT[2] = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.mag_z_idx)]));
        }

        // 5. Baro
        if (mapping.pressure_idx >= 0 && mapping.pressure_idx < static_cast<int>(tokens.size())) {
            frame.pressure_hpa = static_cast<float>(parseDouble(tokens[static_cast<size_t>(mapping.pressure_idx)], static_cast<double>(kStandardPressureHpa)));
            frame.has_baro = true;
        } else {
            frame.has_baro = false;
        }

        frame.has_imu = true;
        dataset.sensor_frames.push_back(frame);
        row_count++;

        // [M11] Cảnh báo rác đầu vào: nếu quá 5% mẫu parse lỗi -> từ chối dataset
        if (row_parse_errors * 20 > row_count && row_count >= 20) {
            return std::nullopt;
        }

        // 6. Ground Truth — [M11] bounds check đầy đủ (trước đây gt_easting/northing
        // truy cập OOB khi row ngắn hơn header -> crash) và bỏ GT của row lỗi
        if (mapping.gt_easting_idx >= 0 && mapping.gt_easting_idx < static_cast<int>(tokens.size()) &&
            mapping.gt_northing_idx >= 0 && mapping.gt_northing_idx < static_cast<int>(tokens.size())) {
            const auto gt_e = tryParseDouble(tokens[static_cast<size_t>(mapping.gt_easting_idx)]);
            const auto gt_n = tryParseDouble(tokens[static_cast<size_t>(mapping.gt_northing_idx)]);
            if (gt_e.has_value() && gt_n.has_value()) {
                EastingNorthingAlt gt_pos{};
                gt_pos.easting_m = gt_e.value();
                gt_pos.northing_m = gt_n.value();
                if (mapping.gt_alt_idx >= 0 && mapping.gt_alt_idx < static_cast<int>(tokens.size())) {
                    gt_pos.altitude_m = parseDouble(tokens[static_cast<size_t>(mapping.gt_alt_idx)]);
                }
                dataset.ground_truth_positions.push_back(gt_pos);
                dataset.ground_truth_timestamps_ns.push_back(frame.timestamp_ns);

                if (mapping.gt_heading_idx >= 0 && mapping.gt_heading_idx < static_cast<int>(tokens.size())) {
                    double hdg = parseDouble(tokens[static_cast<size_t>(mapping.gt_heading_idx)]);
                    if (mapping.gt_heading_is_deg) {
                        hdg *= kDegToRad;  // [M11] gtheadingdeg -> radian
                    }
                    dataset.ground_truth_headings_rad.push_back(hdg);
                }
            }
        }
    }

    if (dataset.sensor_frames.empty()) {
        return std::nullopt;
    }

    return dataset;
}

std::optional<SyntheticDataset> CsvLoader::loadFromFile(
    const std::string& filepath,
    const CsvLoaderConfig& config
) noexcept {
    std::ifstream file(filepath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return std::nullopt;
    }

    const auto size = file.tellg();
    if (size <= 0) {
        return std::nullopt;
    }

    std::string content(static_cast<size_t>(size), '\0');
    file.seekg(0);
    file.read(content.data(), size);

    return loadFromString(content, config);
}

std::optional<SyntheticDataset> CsvLoader::loadSensorLoggerFolder(
    const std::string& folder_path
) noexcept {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::exists(folder_path, ec) || !fs::is_directory(folder_path, ec)) {
        return std::nullopt;
    }

    // 1. Đọc chuỗi Accel (ưu tiên TotalAcceleration.csv, fallback sang Accelerometer.csv)
    std::string accel_file = (fs::exists(fs::path(folder_path) / "TotalAcceleration.csv", ec)) 
        ? (fs::path(folder_path) / "TotalAcceleration.csv").string()
        : (fs::path(folder_path) / "Accelerometer.csv").string();

    auto accel_series = read3DSeries(accel_file);
    if (accel_series.empty()) {
        return std::nullopt;
    }

    // 2. Đọc chuỗi Gyro
    std::string gyro_file = (fs::path(folder_path) / "Gyroscope.csv").string();
    auto gyro_series = read3DSeries(gyro_file);

    // 3. Đọc chuỗi Mag
    std::string mag_file = (fs::path(folder_path) / "Magnetometer.csv").string();
    auto mag_series = read3DSeries(mag_file);

    // 4. Đọc chuỗi Orientation (Yaw góc hướng)
    std::string orientation_file = (fs::path(folder_path) / "Orientation.csv").string();
    auto yaw_series = readScalarSeries(orientation_file, "yaw");

    // 5. Đồng bộ hóa các luồng cảm biến theo mốc thời gian của Accel
    SyntheticDataset dataset;
    dataset.sensor_frames.reserve(accel_series.size());

    size_t gyro_idx = 0;
    size_t mag_idx = 0;
    size_t yaw_idx = 0;

    for (const auto& acc : accel_series) {
        SensorFrame frame{};
        frame.timestamp_ns = acc.timestamp_ns;
        frame.has_imu = true;
        frame.accel_m_s2 = {acc.x, acc.y, acc.z};

        // Tìm mẫu Gyro gần nhất
        while (gyro_idx + 1 < gyro_series.size() && gyro_series[gyro_idx + 1].timestamp_ns <= acc.timestamp_ns) {
            gyro_idx++;
        }
        if (!gyro_series.empty()) {
            frame.gyro_rad_s = {gyro_series[gyro_idx].x, gyro_series[gyro_idx].y, gyro_series[gyro_idx].z};
        }

        // Tìm mẫu Mag gần nhất (trong khoảng 100ms)
        while (mag_idx + 1 < mag_series.size() && mag_series[mag_idx + 1].timestamp_ns <= acc.timestamp_ns) {
            mag_idx++;
        }
        if (!mag_series.empty()) {
            frame.mag_uT = {mag_series[mag_idx].x, mag_series[mag_idx].y, mag_series[mag_idx].z};
            frame.has_mag = true;
        } else {
            frame.has_mag = false;
        }

        frame.has_baro = false; // Sensor Logger dataset này không có barometer
        dataset.sensor_frames.push_back(frame);

        // Đọc ground truth orientation yaw nếu có
        if (!yaw_series.empty()) {
            while (yaw_idx + 1 < yaw_series.size() && yaw_series[yaw_idx + 1].timestamp_ns <= acc.timestamp_ns) {
                yaw_idx++;
            }
            dataset.ground_truth_headings_rad.push_back(yaw_series[yaw_idx].value);
            dataset.ground_truth_timestamps_ns.push_back(acc.timestamp_ns);
        }
    }

    return dataset;
}

} // namespace survival::harness
