#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_harness/csv_loader.hpp"

using namespace survival;
using namespace survival::estimators;
using namespace survival::harness;
using survival::kinematics::FlightPhase;
using survival::kinematics::StairActivity;

const char* activityToString(MotionActivity act) {
    switch (act) {
        case MotionActivity::kStationary: return "STATIONARY";
        case MotionActivity::kWalkingFlat: return "WALKING_FLAT";
        case MotionActivity::kRunning: return "RUNNING";
        case MotionActivity::kStairsAscent: return "STAIRS_ASCENT";
        case MotionActivity::kStairsDescent: return "STAIRS_DESCENT";
        case MotionActivity::kStairLanding: return "STAIR_LANDING";
        case MotionActivity::kEscalator: return "ESCALATOR";
        case MotionActivity::kCycling: return "CYCLING";
        case MotionActivity::kVehicleWheeled: return "VEHICLE_WHEELED";
        case MotionActivity::kElevator: return "ELEVATOR";
        case MotionActivity::kCableTransport: return "CABLE_TRANSPORT";
        default: return "UNKNOWN";
    }
}

struct ActivitySegment {
    MotionActivity activity;
    double start_time_s;
    double end_time_s;
    size_t step_count;
    double avg_speed;
};

void writeCsvHeader(std::ostream& csv) {
    csv << "dataset,steps,duration_s,final_east_m,final_north_m,loop_closure_m,"
           "stair_step_count,riser_m,cumulative_vertical_m,landing_count,"
           "activity,activity_duration_s,activity_steps,activity_avg_speed_m_s\n";
}

void writeCsvRows(std::ostream& csv, const std::string& name, int total_steps, double duration_s,
                  double final_east, double final_north, double loop_closure_m,
                  int stair_steps, double riser_m, double cum_vertical_m, int landings,
                  const std::vector<ActivitySegment>& segments) {
    for (const auto& seg : segments) {
        const double dur = seg.end_time_s - seg.start_time_s;
        if (dur < 0.2 && seg.step_count == 0) continue;
        csv << name << ',' << total_steps << ',' << duration_s << ','
            << final_east << ',' << final_north << ',' << loop_closure_m << ','
            << stair_steps << ',' << riser_m << ',' << cum_vertical_m << ',' << landings << ','
            << activityToString(seg.activity) << ',' << dur << ',' << seg.step_count << ','
            << seg.avg_speed << "\n";
    }
}

void processAndReportDataset(std::ostream& out, const std::string& name, const std::string& folder_path,
                             std::ostream* csv_out = nullptr, std::ostream* probe_out = nullptr) {
    out << "\n======================================================================\n";
    out << ">>> EVALUATING REAL DATASET: " << name << "\n";
    out << ">>> Folder: " << folder_path << "\n";
    out << "======================================================================\n";

    auto dataset_opt = CsvLoader::loadSensorLoggerFolder(folder_path);
    if (!dataset_opt.has_value() || dataset_opt->sensor_frames.empty()) {
        out << "[ERROR] Failed to load dataset from: " << folder_path << "\n";
        return;
    }

    const auto& frames = dataset_opt->sensor_frames;
    out << "[INFO] Loaded " << frames.size() << " synchronized sensor frames.\n";

    MultiModalFusionPipeline pipeline{};
    environment::DemMap empty_dem{};
    pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 100.0}, empty_dem);

    const uint64_t t0 = frames.front().timestamp_ns;

    std::vector<ActivitySegment> segments;
    MotionActivity current_seg_act = MotionActivity::kStationary;
    double seg_start_t = 0.0;
    size_t seg_start_steps = 0;
    double speed_accum = 0.0;
    size_t speed_samples = 0;

    out << "\n--- TIMELINE PROGRESS (Every Step Detected) ---\n";
    out << std::left << std::setw(10) << "Time (s)"
        << std::setw(8)  << "Step #"
        << std::setw(18) << "Activity"
        << std::setw(12) << "Confidence"
        << std::setw(12) << "V_horiz(m/s)"
        << std::setw(12) << "V_z(m/s)"
        << std::setw(12) << "Easting(m)"
        << std::setw(12) << "Northing(m)"
        << std::setw(14) << "Stairs Riser"
        << "\n";
    out << std::string(100, '-') << "\n";

    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        const double t_s = static_cast<double>(frame.timestamp_ns - t0) * 1e-9;

        auto state_opt = pipeline.processSensorFrame(frame);

        const auto& act_est = pipeline.getActivityClassifier().getLastEstimate();
        const auto& vel_est = pipeline.getVelocityEstimator();
        const double current_speed = vel_est.getFilteredSpeed();

        speed_accum += current_speed;
        speed_samples++;

        // Theo dõi đoạn phân loại (Segment Tracking)
        if (segments.empty() || act_est.current_activity != current_seg_act) {
            if (!segments.empty()) {
                segments.back().end_time_s = t_s;
                segments.back().step_count = pipeline.getTotalSteps() - seg_start_steps;
                segments.back().avg_speed = (speed_samples > 0) ? (speed_accum / static_cast<double>(speed_samples)) : 0.0;
            }
            current_seg_act = act_est.current_activity;
            seg_start_t = t_s;
            seg_start_steps = pipeline.getTotalSteps();
            speed_accum = 0.0;
            speed_samples = 0;

            segments.push_back(ActivitySegment{
                .activity = current_seg_act,
                .start_time_s = t_s,
                .end_time_s = t_s,
                .step_count = 0,
                .avg_speed = 0.0
            });
        }

        if (state_opt.has_value()) {
            const auto& st = *state_opt;
            const auto pos = pipeline.getDeadReckoningPosition();
            const auto stair_state = pipeline.getActivityClassifier().getStairState();
            const auto& sm = pipeline.getActivityClassifier().getStairMotionState();

            if (probe_out && probe_out->good()) {
                const char* phase_str = (sm.phase == FlightPhase::kConfirmed) ? "CONFIRMED"
                    : (sm.phase == FlightPhase::kCandidate) ? "CANDIDATE"
                    : (sm.phase == FlightPhase::kLandingTransition) ? "LANDING" : "UNKNOWN";
                const char* act_str = (sm.activity == StairActivity::kAscent) ? "UP"
                    : (sm.activity == StairActivity::kDescent) ? "DOWN"
                    : (sm.activity == StairActivity::kLanding) ? "LANDING"
                    : (sm.activity == StairActivity::kPaused) ? "PAUSED" : "NONE";
                (*probe_out) << t_s << ',' << pipeline.getTotalSteps() << ','
                             << st.activity_confidence << ','
                             << sm.p_stair << ',' << sm.p_ascent << ',' << sm.p_descent << ','
                             << sm.last_step_delta_z_m << ',' << sm.direction_confidence << ','
                             << sm.last_rise_time_s << ',' << sm.last_skewness << ','
                             << sm.last_v_to_h_ratio << ',' << sm.last_peak_to_peak_m_s2 << ','
                             << sm.last_freefall_ms << ','
                             << phase_str << ',' << act_str << "\n";
            }

            out << std::fixed << std::setprecision(2)
                << std::left << std::setw(10) << t_s
                << std::setw(8)  << pipeline.getTotalSteps()
                << std::setw(18) << activityToString(st.motion_activity)
                << std::setw(12) << st.activity_confidence
                << std::setw(12) << st.instantaneous_speed_m_s
                << std::setw(12) << st.vertical_speed_m_s
                << std::setw(12) << pos.easting_m
                << std::setw(12) << pos.northing_m;
            if (stair_state.step_count > 0) {
                out << std::setw(14) << stair_state.estimated_riser_m;
            } else {
                out << std::setw(14) << "N/A";
            }
            out << "\n";
        }
    }

    if (!segments.empty()) {
        const double final_t = static_cast<double>(frames.back().timestamp_ns - t0) * 1e-9;
        segments.back().end_time_s = final_t;
        segments.back().step_count = pipeline.getTotalSteps() - seg_start_steps;
        segments.back().avg_speed = (speed_samples > 0) ? (speed_accum / static_cast<double>(speed_samples)) : 0.0;
    }

    const auto final_pos = pipeline.getDeadReckoningPosition();
    const double net_dist = std::sqrt(final_pos.easting_m * final_pos.easting_m + final_pos.northing_m * final_pos.northing_m);
    const auto stair_summary = pipeline.getActivityClassifier().getStairState();

    out << "\n======================================================================\n";
    out << ">>> KẾT QUẢ TỔNG KẾT PHÂN TÍCH HÀNH VI (BIOMECHANICAL & MOTION SUMMARY)\n";
    out << "======================================================================\n";
    out << "1. Tổng thời gian: " << std::fixed << std::setprecision(1) << (static_cast<double>(frames.back().timestamp_ns - t0) * 1e-9) << " giây\n";
    out << "2. Tổng số bước nhận diện: " << pipeline.getTotalSteps() << " bước\n";
    out << "3. Tọa độ điểm cuối (ENU Dead Reckoning): East = " << std::setprecision(2) << final_pos.easting_m 
        << " m, North = " << final_pos.northing_m << " m\n";
    out << "4. Độ lệch khép kín vòng lặp (Loop closure error): " << net_dist << " m\n";
    out << "5. Thống kê Leo Cầu Thang:\n";
    out << "   - Tổng số bậc cầu thang: " << stair_summary.step_count << " bậc\n";
    out << "   - Chiều cao mỗi bậc ước lượng: " << stair_summary.estimated_riser_m << " m\n";
    out << "   - Độ cao thẳng đứng tích lũy: " << stair_summary.cumulative_vertical_m << " m\n";
    out << "   - Số bước trên chiếu nghỉ (Landings): " << stair_summary.landing_count << " bước\n";

    if (csv_out && csv_out->good()) {
        const double duration_s = static_cast<double>(frames.back().timestamp_ns - t0) * 1e-9;
        writeCsvRows(*csv_out, name, pipeline.getTotalSteps(), duration_s,
                     final_pos.easting_m, final_pos.northing_m, net_dist,
                     stair_summary.step_count, stair_summary.estimated_riser_m,
                     stair_summary.cumulative_vertical_m, stair_summary.landing_count,
                     segments);
    }

    out << "\n6. Bảng phân rã các phân đoạn hoạt động (Activity Breakdown Timeline):\n";
    out << std::left << std::setw(18) << "Hoạt động"
        << std::setw(15) << "Bắt đầu (s)"
        << std::setw(15) << "Kết thúc (s)"
        << std::setw(15) << "Thời lượng(s)"
        << std::setw(12) << "Số bước"
        << std::setw(15) << "Tốc độ TB (m/s)"
        << "\n";
    out << std::string(85, '-') << "\n";

    for (const auto& seg : segments) {
        const double duration = seg.end_time_s - seg.start_time_s;
        if (duration < 0.2 && seg.step_count == 0) continue;

        out << std::left << std::setw(18) << activityToString(seg.activity)
            << std::setw(15) << seg.start_time_s
            << std::setw(15) << seg.end_time_s
            << std::setw(15) << duration
            << std::setw(12) << seg.step_count
            << std::setw(15) << seg.avg_speed
            << "\n";
    }
    out << "======================================================================\n\n";
}

// Usage: replay_evaluation [dataset_folder] [--csv <out.csv>] [--out <out.txt>] [--probe <probe.csv>]
// Không đối số: đánh giá 2 dataset mặc định, in ra stdout.
// --csv: ghi thêm bản tóm tắt machine-readable (mỗi dòng = 1 phân đoạn hoạt động).
// --probe: dump đặc trưng hướng thang từng bước (p_ascent/p_descent/delta-Z...) để chẩn đoán cực tính.
int main(int argc, char* argv[]) {
    std::string dataset;
    std::string out_path;
    std::string csv_path;
    std::string probe_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--csv" && i + 1 < argc) csv_path = argv[++i];
        else if (arg == "--out" && i + 1 < argc) out_path = argv[++i];
        else if (arg == "--probe" && i + 1 < argc) probe_path = argv[++i];
        else dataset = arg;
    }

    std::ofstream file_out;
    std::ostream* out = &std::cout;
    if (!out_path.empty()) {
        file_out.open(out_path);
        if (!file_out.is_open()) {
            std::cerr << "[ERROR] Cannot open output file: " << out_path << " — falling back to stdout\n";
        } else {
            out = &file_out;
        }
    }

    std::ofstream csv_out;
    if (!csv_path.empty()) {
        csv_out.open(csv_path);
        if (!csv_out.is_open()) {
            std::cerr << "[ERROR] Cannot open CSV file: " << csv_path << "\n";
        } else {
            writeCsvHeader(csv_out);
        }
    }

    std::ofstream probe_out;
    if (!probe_path.empty()) {
        probe_out.open(probe_path);
        if (!probe_out.is_open()) {
            std::cerr << "[ERROR] Cannot open probe file: " << probe_path << "\n";
        } else {
            probe_out << "t_s,step,activity_conf,p_stair,p_ascent,p_descent,delta_z_m,dir_conf,rise_time_s,skewness,v_to_h,p2p,freefall_ms,phase,stair_act\n";
        }
    }
    std::ostream* probe_ptr = probe_out.is_open() ? &probe_out : nullptr;
    std::ostream* csv_ptr = csv_out.is_open() ? &csv_out : nullptr;

    if (!dataset.empty()) {
        processAndReportDataset(*out, dataset, dataset, csv_ptr, probe_ptr);
    } else {
        processAndReportDataset(*out, "Dataset 1: 2026-09-01_10-57-01_1 (Stair Ascent & Walking Loop)",
                                "C:/Users/KieuKhang/Downloads/2026-09-01_10-57-01_1", csv_ptr, probe_ptr);
        processAndReportDataset(*out, "Dataset 2: 2026-09-01_11-24-39_1 (Jogging / Fast Walking Loop)",
                                "C:/Users/KieuKhang/Downloads/2026-09-01_11-24-39_1", csv_ptr, probe_ptr);
    }

    if (file_out.is_open()) file_out.close();
    if (csv_out.is_open()) csv_out.close();
    if (probe_out.is_open()) probe_out.close();
    return 0;
}
