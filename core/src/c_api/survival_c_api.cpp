#include "survival_core/c_api/survival_c_api.h"
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/environment/map_manager.hpp"
#include "survival_core/safety/safety_auditor.hpp"
#include "survival_core/safety/resource_governor.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include "survival_core/comms/sos_egress_manager.hpp"
#include "survival_core/storage/local_blackbox.hpp"
#include "survival_platform/platform_factory.hpp"
#include <atomic>
#include <cstring>
#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>

namespace {

/**
 * @brief Triple Buffer cho trạng thái điều hướng (1 Writer / N Readers)
 * Đảm bảo Bất biến Sở hữu Độc quyền: READ != WRITE != CLEAN
 * Ngữ nghĩa đồng bộ hóa: Writer release -> Reader acquire qua 1 biến atomic 8-bit.
 *
 * [H5] Writer path giữ nguyên 100% lock-free. Các READER được tuần tự hóa với
 * NHAU bằng mutex riêng (3 entry point C-API cùng đọc: uncertainty/ble/sms có thể
 * chạy trên 3 thread khác nhau) — writer không bao giờ chạm mutex này.
 */
template <typename T>
class LockFreeTripleBuffer {
private:
    std::array<T, 3> buffers_{};
    // 1 biến atomic duy nhất quản lý chỉ số buffer sạch và cờ dữ liệu mới:
    // Bit 0..1: clean_idx (0, 1, 2)
    // Bit 2:    has_new_data (0 hoặc 1)
    mutable std::atomic<uint8_t> shared_state_{0x00}; // clean = 0, no new data
    uint8_t write_idx_{1}; // Độc quyền sở hữu bởi Writer thread
    mutable std::mutex readers_mutex_; // Tuần tự hóa các reader với nhau (không chạm writer)
    mutable uint8_t read_idx_{2};  // Chỉ bị đụng tới khi đang giữ readers_mutex_

    static_assert(std::atomic<uint8_t>::is_always_lock_free, "uint8_t atomic must be lock-free on all platforms");

public:
    LockFreeTripleBuffer() noexcept = default;

    void write(const T& val) noexcept {
        buffers_[write_idx_] = val; // Ghi an toàn vào buffer độc quyền của Writer

        // Công bố buffer vừa ghi và lấy về clean_idx cũ
        uint8_t old_state = shared_state_.load(std::memory_order_relaxed);
        uint8_t new_state = 0;
        do {
            new_state = static_cast<uint8_t>((write_idx_ & 0x03) | 0x04); // clean = write_idx, has_new = 1
        } while (!shared_state_.compare_exchange_weak(
            old_state, new_state,
            std::memory_order_release,
            std::memory_order_relaxed));

        write_idx_ = static_cast<uint8_t>(old_state & 0x03); // Writer nhận clean buffer cũ làm write buffer mới
    }

    bool read(T& out_val) const noexcept {
        // [H5] N-reader: khóa reader-only, bảo vệ read_idx_ và CAS publish
        std::lock_guard<std::mutex> lock(readers_mutex_);
        uint8_t old_state = shared_state_.load(std::memory_order_relaxed);
        if ((old_state & 0x04) != 0) { // Có dữ liệu mới được publish
            uint8_t new_state = 0;
            do {
                if ((old_state & 0x04) == 0) break;
                new_state = static_cast<uint8_t>(read_idx_ & 0x03); // clean = read_idx, has_new = 0
            } while (!shared_state_.compare_exchange_weak(
                old_state, new_state,
                std::memory_order_acquire,
                std::memory_order_relaxed));

            if ((old_state & 0x04) != 0) {
                read_idx_ = static_cast<uint8_t>(old_state & 0x03); // Reader nhận published buffer làm read buffer mới
            }
        }
        out_val = buffers_[read_idx_]; // Đọc an toàn: writer không thể sở hữu buffer này
        return true;
    }
};

struct EngineContext {
    survival::estimators::MultiModalFusionPipeline pipeline{};
    survival::environment::DemMap dem_map{};
    survival::environment::MapManager map_manager{};
    survival::safety::SafetyAuditor safety_auditor{};
    survival::safety::ResourceGovernor resource_governor{};
    survival::storage::LocalBlackbox blackbox{};

    // Triple Buffer cho trạng thái điều hướng (1 Writer / N Readers)
    LockFreeTripleBuffer<survival::NavigationUncertaintyState> shared_state{};
    // [H5] Các trạng thái dùng chéo luồng bắt buộc atomic (header hứa "Thread-safe
    // snapshot"): sensor thread ghi, các thread UI/BLE/SMS/battery đọc.
    std::atomic<double> current_battery_soc{100.0};
    std::atomic<uint64_t> last_sensor_frame_mono_ns{0};
    std::atomic<uint64_t> simulated_offset_ns{0};

    // Theo dõi độ dịch chuyển và tổng quãng đường cho Panic Circling
    // (chỉ dùng trong luồng sensor — không cần atomic)
    survival::EastingNorthingAlt start_pdr_pos{0.0, 0.0, 1000.0};
    survival::EastingNorthingAlt prev_pdr_pos{0.0, 0.0, 1000.0};
    double accumulated_distance_m{0.0};
    bool has_start_pos{false};
};

} // namespace

extern "C" {

survival_handle_t survival_engine_create(void) {
    // [M15] Đăng ký platform hooks (fsync blackbox / mmap DEM) — idempotent.
    // Trước đây chỉ test gọi nên production chạy không có durability.
    survival::platform::initializePlatformHooks();
    auto* ctx = new (std::nothrow) EngineContext();
    return static_cast<survival_handle_t>(ctx);
}

void survival_engine_destroy(survival_handle_t handle) {
    if (handle != nullptr) {
        auto* ctx = static_cast<EngineContext*>(handle);
        delete ctx;
    }
}

int survival_engine_load_offline_dem(survival_handle_t handle, const char* filepath) {
    if (handle == nullptr || filepath == nullptr) {
        return 0;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    const bool ok = ctx->dem_map.loadFromFile(filepath, false);
    if (ok) {
        survival::LocalFrame frame{
            .origin_lat_rad = ctx->dem_map.getHeader().origin_lat_rad,
            .origin_lon_rad = ctx->dem_map.getHeader().origin_lon_rad,
            .origin_alt_m = 0.0
        };
        ctx->map_manager.setLocalFrame(frame);
        ctx->pipeline.initialize(survival::EastingNorthingAlt{0.0, 0.0, 1000.0}, ctx->dem_map);
        return 1;
    }
    return 0;
}

void survival_engine_push_gps_fix(
    survival_handle_t handle,
    double lat_rad,
    double lon_rad,
    double alt_m,
    double std_m,
    double hdop,
    uint64_t timestamp_ns) {
    
    if (handle == nullptr) {
        return;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    survival::environment::GpsSample sample{};
    sample.lat_rad = lat_rad;
    sample.lon_rad = lon_rad;
    sample.alt_m = alt_m;
    sample.std_pos_m = std_m;
    sample.hdop = hdop;
    sample.timestamp_ns = timestamp_ns;
    sample.is_valid = true;

    ctx->map_manager.handleGpsFix(sample, ctx->pipeline.getFilteredBaroAltitude());
}

void survival_engine_set_declination(survival_handle_t handle, double declination_deg) {
    if (handle == nullptr) {
        return;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    ctx->pipeline.setMagDeclinationRad(declination_deg * survival::kDegToRad);
}

void survival_engine_push_gps_cutoff(survival_handle_t handle, uint64_t timestamp_ns) {
    if (handle == nullptr) {
        return;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    const auto handoff = ctx->map_manager.handleGpsCutoff(timestamp_ns);
    if (handoff.is_valid) {
        // Tái lập điểm neo và lan truyền bảo toàn ngữ cảnh sinh học & bước chân
        ctx->pipeline.updateAnchorPosition(handoff.anchor_pos, handoff.anchor_std_m);
    }
}

int survival_engine_push_sensor_frame(survival_handle_t handle, const void* sensor_frame_ptr) {
    if (handle == nullptr || sensor_frame_ptr == nullptr) {
        return 0;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    const auto* frame = static_cast<const survival::SensorFrame*>(sensor_frame_ptr);

    // Ghi nhận thời gian cập nhật cảm biến (Freshness Watchdog) — atomic [H5]
    ctx->last_sensor_frame_mono_ns.store(
        static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count()),
        std::memory_order_relaxed);

    // 1. Cập nhật Safety Auditor (100 Hz Motion)
    const survival::Vector3f accel{frame->accel_m_s2[0], frame->accel_m_s2[1], frame->accel_m_s2[2]};
    const survival::Vector3f gyro{frame->gyro_rad_s[0], frame->gyro_rad_s[1], frame->gyro_rad_s[2]};
    ctx->safety_auditor.processMotionSample(accel, gyro, frame->timestamp_ns);

    // 2. Chạy Pipeline Hợp nhất đa phương thức
    auto state_opt = ctx->pipeline.processSensorFrame(*frame);
    if (state_opt.has_value()) {
        auto& state = *state_opt;

        // Cập nhật vị trí và quãng đường tích lũy cho Safety Auditor (Panic Circling)
        const auto pdr_pos = ctx->pipeline.getDeadReckoningPosition();
        if (!ctx->has_start_pos) {
            ctx->start_pdr_pos = pdr_pos;
            ctx->prev_pdr_pos = pdr_pos;
            ctx->has_start_pos = true;
        } else {
            const double d_step = std::hypot(pdr_pos.easting_m - ctx->prev_pdr_pos.easting_m,
                                             pdr_pos.northing_m - ctx->prev_pdr_pos.northing_m);
            ctx->accumulated_distance_m += d_step;
            ctx->prev_pdr_pos = pdr_pos;
        }

        const double net_displacement = std::hypot(pdr_pos.easting_m - ctx->start_pdr_pos.easting_m,
                                                   pdr_pos.northing_m - ctx->start_pdr_pos.northing_m);
        const size_t total_steps = ctx->pipeline.getTotalSteps();
        ctx->safety_auditor.updateOdometryDisplacement(net_displacement, ctx->accumulated_distance_m, total_steps);

        // Cập nhật trạng thái an toàn
        state.safety_state = ctx->safety_auditor.getState();
        state.flags.distress_sos_active = ctx->safety_auditor.isDistressActive();
        state.flags.panic_circling = ctx->safety_auditor.isPanicCirclingWarning();

        // Ghi vào Lock-Free Triple Buffer
        ctx->shared_state.write(state);

        // Ghi vào Blackbox Storage
        survival::storage::BlackboxRecord rec{};
        rec.timestamp_ns = frame->timestamp_ns;
        rec.easting_m = static_cast<float>(state.clusters[0].center_easting_m);
        rec.northing_m = static_cast<float>(state.clusters[0].center_northing_m);
        rec.altitude_m = static_cast<float>(state.clusters[0].center_altitude_m);
        rec.ce95_m = static_cast<float>(state.clusters[0].ce95_semi_major_m);
        rec.heading_deg = static_cast<float>(state.clusters[0].ellipse_heading_rad * survival::kRadToDeg);
        rec.battery_soc = static_cast<float>(ctx->current_battery_soc.load(std::memory_order_relaxed));
        rec.safety_state = static_cast<uint8_t>(state.safety_state);
        rec.flags = 0;
        ctx->blackbox.pushRecord(rec);

        return 1;
    }

    return 0;
}

int survival_engine_get_uncertainty_state(survival_handle_t handle, void* out_state_ptr) {
    if (handle == nullptr || out_state_ptr == nullptr) {
        return 0;
    }
    const auto* ctx = static_cast<const EngineContext*>(handle);
    auto* out_state = static_cast<survival::NavigationUncertaintyState*>(out_state_ptr);
    return ctx->shared_state.read(*out_state) ? 1 : 0;
}

int survival_engine_get_motion_activity(survival_handle_t handle, uint8_t* out_activity, float* out_confidence) {
    if (handle == nullptr || out_activity == nullptr) {
        return 0;
    }
    const auto* ctx = static_cast<const EngineContext*>(handle);
    const auto& est = ctx->pipeline.getActivityClassifier().getLastEstimate();
    *out_activity = static_cast<uint8_t>(est.current_activity);
    if (out_confidence != nullptr) {
        *out_confidence = est.fused_confidence;
    }
    return 1;
}

int survival_engine_get_instantaneous_velocity(
    survival_handle_t handle,
    double* out_vx,
    double* out_vy,
    double* out_vz,
    double* out_speed
) {
    if (handle == nullptr) {
        return 0;
    }
    const auto* ctx = static_cast<const EngineContext*>(handle);
    const double speed = ctx->pipeline.getVelocityEstimator().getFilteredSpeed();
    const double yaw = ctx->pipeline.getEskf().getHeadingRad();
    if (out_vx != nullptr) *out_vx = speed * std::sin(yaw);
    if (out_vy != nullptr) *out_vy = speed * std::cos(yaw);
    if (out_vz != nullptr) *out_vz = 0.0;
    if (out_speed != nullptr) *out_speed = speed;
    return 1;
}

int survival_engine_get_ble_adv_payload(
    survival_handle_t handle,
    uint8_t* out_buffer,
    size_t max_len,
    size_t* out_len) {
    
    if (handle == nullptr || out_buffer == nullptr || out_len == nullptr || max_len < 24) {
        return 0;
    }

    const auto* ctx = static_cast<const EngineContext*>(handle);
    survival::NavigationUncertaintyState current_state{};
    if (!ctx->shared_state.read(current_state)) {
        current_state.clusters[0].center_altitude_m = 1000.0;
        current_state.clusters[0].ce95_semi_major_m = 50.0;
        current_state.clusters[0].ce95_semi_minor_m = 30.0;
    }

    double lat_deg = 0.0;
    double lon_deg = 0.0;
    const auto& frame = ctx->map_manager.getLocalFrame();
    if (frame.origin_lat_rad != 0.0 || frame.origin_lon_rad != 0.0) {
        const survival::EastingNorthingAlt enu{
            .easting_m = current_state.clusters[0].center_easting_m,
            .northing_m = current_state.clusters[0].center_northing_m,
            .altitude_m = current_state.clusters[0].center_altitude_m
        };
        const auto geo = frame.enuToGeodetic(enu);
        lat_deg = geo.latitude_deg;
        lon_deg = geo.longitude_deg;
    } else {
        const auto& fix = ctx->map_manager.getLastKnownGoodFix();
        if (fix.is_valid) {
            lat_deg = fix.latitude_deg;
            lon_deg = fix.longitude_deg;
        }
    }

    // [M9] Bit NO_FIX (0x80): không có datum DEM và không có LastKnownGoodFix
    const bool has_fix = (frame.origin_lat_rad != 0.0 || frame.origin_lon_rad != 0.0) ||
                         ctx->map_manager.getLastKnownGoodFix().is_valid;
    const uint8_t rescue_status_flags = static_cast<uint8_t>(current_state.nav_mode) |
                                        (has_fix ? 0x00U : 0x80U);
    const auto raw_bytes = survival::comms::Packet160Serializer::encode(
        static_cast<uint32_t>(current_state.timestamp_ms / 1000),
        lat_deg,
        lon_deg,
        current_state.clusters[0].center_altitude_m,
        current_state.clusters[0].ce95_semi_major_m * 0.5,
        current_state.clusters[0].ce95_semi_minor_m * 0.5,
        current_state.clusters[0].ellipse_heading_rad * survival::kRadToDeg,
        rescue_status_flags,
        static_cast<uint8_t>(ctx->current_battery_soc.load(std::memory_order_relaxed)),
        current_state.clusters[0].cluster_id
    );

    survival::OutboundRescuePacket160 packet{};
    packet.data = raw_bytes;

    const auto ble_adv = survival::comms::SosEgressManager::buildBleAdvPayload(packet);
    if (ble_adv.length > max_len) {
        return 0;
    }

    std::memcpy(out_buffer, ble_adv.raw_adv_data.data(), ble_adv.length);
    *out_len = ble_adv.length;
    return 1;
}

int survival_engine_get_sms_payload(
    survival_handle_t handle,
    char* out_str,
    size_t max_len) {
    
    if (handle == nullptr || out_str == nullptr || max_len == 0) {
        return 0;
    }

    const auto* ctx = static_cast<const EngineContext*>(handle);
    survival::NavigationUncertaintyState current_state{};
    if (!ctx->shared_state.read(current_state)) {
        current_state.clusters[0].center_altitude_m = 1000.0;
        current_state.clusters[0].ce95_semi_major_m = 50.0;
        current_state.clusters[0].ce95_semi_minor_m = 30.0;
    }

    double lat_deg = 0.0;
    double lon_deg = 0.0;
    const auto& frame = ctx->map_manager.getLocalFrame();
    if (frame.origin_lat_rad != 0.0 || frame.origin_lon_rad != 0.0) {
        const survival::EastingNorthingAlt enu{
            .easting_m = current_state.clusters[0].center_easting_m,
            .northing_m = current_state.clusters[0].center_northing_m,
            .altitude_m = current_state.clusters[0].center_altitude_m
        };
        const auto geo = frame.enuToGeodetic(enu);
        lat_deg = geo.latitude_deg;
        lon_deg = geo.longitude_deg;
    } else {
        const auto& fix = ctx->map_manager.getLastKnownGoodFix();
        if (fix.is_valid) {
            lat_deg = fix.latitude_deg;
            lon_deg = fix.longitude_deg;
        }
    }

    // [M9] Bit NO_FIX (0x80): không có datum DEM và không có LastKnownGoodFix
    const bool has_fix = (frame.origin_lat_rad != 0.0 || frame.origin_lon_rad != 0.0) ||
                         ctx->map_manager.getLastKnownGoodFix().is_valid;
    const uint8_t rescue_status_flags = static_cast<uint8_t>(current_state.nav_mode) |
                                        (has_fix ? 0x00U : 0x80U);
    const auto raw_bytes = survival::comms::Packet160Serializer::encode(
        static_cast<uint32_t>(current_state.timestamp_ms / 1000),
        lat_deg,
        lon_deg,
        current_state.clusters[0].center_altitude_m,
        current_state.clusters[0].ce95_semi_major_m * 0.5,
        current_state.clusters[0].ce95_semi_minor_m * 0.5,
        current_state.clusters[0].ellipse_heading_rad * survival::kRadToDeg,
        rescue_status_flags,
        static_cast<uint8_t>(ctx->current_battery_soc.load(std::memory_order_relaxed)),
        current_state.clusters[0].cluster_id
    );

    survival::OutboundRescuePacket160 packet{};
    packet.data = raw_bytes;

    const auto sms = survival::comms::SosEgressManager::buildCompactSmsPayload(packet, current_state, ctx->current_battery_soc.load(std::memory_order_relaxed));
    if (sms.length() >= max_len) {
        return 0;
    }

    std::memcpy(out_str, sms.c_str(), sms.length() + 1);
    return 1;
}

void survival_engine_update_battery_soc(survival_handle_t handle, double soc_percent) {
    if (handle == nullptr) {
        return;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    ctx->current_battery_soc.store(soc_percent, std::memory_order_relaxed);
    const auto gov = ctx->resource_governor.updateBatterySoc(soc_percent);
    if (gov.should_trigger_disk_flush) {
        ctx->blackbox.flushToDisk();
    }

    // Adaptive Compute Governor: điều tiết số lượng hạt theo mức pin với sàn bảo vệ ESS
    size_t target_particles = 10000;
    if (soc_percent < 5.0) {
        target_particles = 1500;
    } else if (soc_percent < 15.0) {
        target_particles = 4000;
    }
    if (ctx->pipeline.getEffectiveParticleCount() < 500.0) {
        target_particles = std::max(target_particles, static_cast<size_t>(3000));
    }
    ctx->pipeline.setTargetParticleCount(target_particles);
}

void survival_engine_cancel_safety_alert(survival_handle_t handle) {
    if (handle == nullptr) {
        return;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    ctx->safety_auditor.cancelAlert();
}

int survival_engine_get_safety_state(survival_handle_t handle) {
    if (handle == nullptr) {
        return 0;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    return static_cast<int>(ctx->safety_auditor.getState());
}

int survival_engine_get_integrity_status(
    survival_handle_t handle,
    int* out_is_healthy,
    uint32_t* out_fault_flags
) {
    if (handle == nullptr) {
        return 0;
    }
    auto* ctx = static_cast<EngineContext*>(handle);
    const auto& status = ctx->pipeline.getLatestIntegrityStatus();
    int is_healthy = status.is_healthy ? 1 : 0;
    uint32_t fault_flags = status.fault_flags;

    // Multi-tier Freshness Watchdog (Check against monotonic clock + simulated offset)
    const uint64_t last_frame_mono = ctx->last_sensor_frame_mono_ns.load(std::memory_order_acquire);
    if (last_frame_mono > 0) {
        const uint64_t now_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count())
            + ctx->simulated_offset_ns.load(std::memory_order_acquire);
        if (now_ns > last_frame_mono) {
            const uint64_t elapsed_ns = now_ns - last_frame_mono;
            if (elapsed_ns > 5000000000ULL) { // > 5s: DEAD
                is_healthy = 0;
                fault_flags |= (survival::integrity::kFaultEstimatorStale | survival::integrity::kFaultSensorDead);
            } else if (elapsed_ns > 1500000000ULL) { // > 1.5s: STALE
                is_healthy = 0;
                fault_flags |= survival::integrity::kFaultEstimatorStale;
            } else if (elapsed_ns > 500000000ULL) { // 500ms - 1.5s: DEGRADED
                fault_flags |= survival::integrity::kFaultEstimatorDegraded;
            }
        }
    }

    if (out_is_healthy != nullptr) {
        *out_is_healthy = is_healthy;
    }
    if (out_fault_flags != nullptr) {
        *out_fault_flags = fault_flags;
    }
    return 1;
}

void survival_engine_advance_simulated_time(survival_handle_t handle, uint64_t delta_ns) {
    if (handle != nullptr) {
        auto* ctx = static_cast<EngineContext*>(handle);
        ctx->simulated_offset_ns.fetch_add(delta_ns, std::memory_order_release);
    }
}

} // extern "C"
