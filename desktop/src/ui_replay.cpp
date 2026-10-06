// ============================================================================
// REPLAY PANEL (U2) — phát lại dataset thật qua pipeline, nhìn được kết quả
// ============================================================================
#include "imgui.h"
#include "ui_panels.hpp"
#include "ui_theme.hpp"
#include "survival_harness/csv_loader.hpp"
#include "survival_harness/data_player.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace desktop {

namespace {

struct ReplaySession {
    SyntheticDataset dataset{};
    DataPlayer player{};
    MultiModalFusionPipeline pipeline{};
    DemMap dem{};

    bool loaded{false};
    std::string folder_name;
    std::string true_label;
    bool playing{false};
    double speed{2.0};
    double frame_accum{0.0};

    size_t total_frames{0};
    size_t total_steps{0};
    double start_east{0.0};
    double start_north{0.0};
    NavigationUncertaintyState latest{};

    // Timeline hoạt động: (t_s, activity) lấy mẫu 1Hz
    std::deque<std::pair<double, uint8_t>> activity_timeline;
    double last_timeline_t{-10.0};
};

ReplaySession& session() {
    static ReplaySession s{};
    return s;
}

void processReplayFrame(const SensorFrame& frame) {
    auto& rs = session();
    const auto state_opt = rs.pipeline.processSensorFrame(frame);
    if (state_opt.has_value()) {
        rs.latest = *state_opt;
    }
    const double t_s = static_cast<double>(frame.timestamp_ns) * 1e-9;
    if (t_s - rs.last_timeline_t >= 1.0) {
        rs.last_timeline_t = t_s;
        rs.activity_timeline.emplace_back(t_s, static_cast<uint8_t>(rs.latest.motion_activity));
        while (rs.activity_timeline.size() > 600) rs.activity_timeline.pop_front();
    }
}

std::vector<std::pair<std::string, std::string>> listPresets() {
    std::vector<std::pair<std::string, std::string>> presets;
    // 2 dataset tự thu (nếu có)
    for (const char* d : {"C:/Users/KieuKhang/Downloads/2026-09-01_10-57-01_1",
                          "C:/Users/KieuKhang/Downloads/2026-09-01_11-24-39_1"}) {
        if (std::filesystem::exists(d)) {
            presets.emplace_back(d, std::filesystem::path(d).filename().string());
        }
    }
    // MotionSense manifest (nếu có): "folder,label"
    const std::filesystem::path manifest =
        std::filesystem::path("../testdata/motionsense/manifest.csv");
    std::ifstream fh(manifest);
    if (fh.is_open()) {
        std::string line;
        std::getline(fh, line);  // header
        int count = 0;
        while (std::getline(fh, line) && count < 64) {
            auto comma = line.find(',');
            auto comma2 = line.find(',', comma + 1);
            if (comma == std::string::npos || comma2 == std::string::npos) continue;
            const std::string folder = line.substr(0, comma);
            const std::string label = line.substr(comma + 1, comma2 - comma - 1);
            std::filesystem::path full = manifest.parent_path() / folder;
            if (std::filesystem::exists(full)) {
                presets.emplace_back(full.string(), folder + " [" + label + "]");
                count++;
            }
        }
    }
    return presets;
}


} // namespace

void renderReplayPanel(AppState& app) {
    (void)app;
    using namespace theme;
    auto& rs = session();

    if (!ImGui::Begin(kWinReplay)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    // ---- Chọn dataset ----
    sectionHeader("DATASET", "NGUỒN DỮ LIỆU");
    static int preset_idx = -1;
    const auto presets = listPresets();
    ImGui::SetNextItemWidth(-90.0F);
    if (!presets.empty()) {
        std::string items;
        for (const auto& [path, label] : presets) {
            items += label + "\0";
        }
        items += "\0";
        if (ImGui::Combo("PRESET", &preset_idx, items.c_str())) {
            snprintf(session().folder_name.data(), 0, "%s", "");
        }
    }
    static char folder_buf[512] = "";
    ImGui::SetNextItemWidth(-90.0F);
    ImGui::InputText("FOLDER", folder_buf, sizeof(folder_buf));

    ImGui::PushStyleColor(ImGuiCol_Border, v4(kAmber));
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
    const bool load_pressed = ImGui::Button("[ NẠP & RESET ]");
    ImGui::PopStyleColor(2);
    if (load_pressed) {
        std::string folder;
        if (preset_idx >= 0 && preset_idx < static_cast<int>(presets.size())) {
            folder = presets[static_cast<size_t>(preset_idx)].first;
            snprintf(folder_buf, sizeof(folder_buf), "%s", folder.c_str());
        } else {
            folder = folder_buf;
        }
        auto dataset = CsvLoader::loadSensorLoggerFolder(folder);
        if (dataset.has_value() && !dataset->sensor_frames.empty()) {
            rs.dataset = std::move(*dataset);
            rs.total_frames = rs.dataset.sensor_frames.size();
            rs.pipeline = MultiModalFusionPipeline{};
            rs.dem = DemMap{};
            rs.pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 100.0}, rs.dem);
            rs.player = DataPlayer{};
            rs.player.setSensorCallback(processReplayFrame);
            rs.player.loadDataset(rs.dataset);
            rs.loaded = true;
            rs.playing = false;
            rs.speed = 2.0;
            rs.total_steps = 0;
            rs.frame_accum = 0.0;
            rs.latest = NavigationUncertaintyState{};
            rs.activity_timeline.clear();
            rs.last_timeline_t = -10.0;
            rs.folder_name = folder;
        }
    }

    if (!rs.loaded) {
        ImGui::Dummy(ImVec2(0, 6));
        smallLabel("NO DATA · Chưa nạp dataset. Chọn preset hoặc gõ đường dẫn thư mục Sensor Logger.", kMuted);
        ImGui::End();
        return;
    }

    // ---- Điều khiển phát ----
    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("TRANSPORT", "ĐIỀU KHIỂN PHÁT");
    const double frames_per_second = 100.0;  // dataset 50-100Hz; dùng 100 chung
    ImGui::PushStyleColor(ImGuiCol_Border, v4(kAmber));
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
    if (ImGui::Button(rs.playing ? "[ ‖ PAUSE ]" : "[ ▶ PLAY ]")) {
        rs.playing = !rs.playing;
    }
    ImGui::PopStyleColor(2);
    ImGui::SameLine();
    if (ImGui::Button("[ +1s ]")) {
        rs.player.step(static_cast<size_t>(frames_per_second));
        rs.playing = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("[ ◀◀ RESTART ]")) {
        rs.player.reset();
        rs.pipeline = MultiModalFusionPipeline{};
        rs.pipeline.initialize(EastingNorthingAlt{0.0, 0.0, 100.0}, rs.dem);
        rs.total_steps = 0;
        rs.latest = NavigationUncertaintyState{};
        rs.activity_timeline.clear();
        rs.last_timeline_t = -10.0;
        rs.playing = false;
    }
    ImGui::SameLine();
    {
        float speed_f = static_cast<float>(rs.speed);
        ImGui::SetNextItemWidth(-90.0F);
        if (ImGui::SliderFloat("TỐC ĐỘ", &speed_f, 0.5F, 8.0F, "×%.1f", ImGuiSliderFlags_AlwaysClamp)) {
            rs.speed = speed_f;
        }
    }

    const float progress = (rs.total_frames > 0)
        ? static_cast<float>(static_cast<double>(rs.player.getCurrentFrameIndex())
                             / static_cast<double>(rs.total_frames))
        : 0.0F;
    segmentedMeter(progress, 40, kAmber, ImGui::GetContentRegionAvail().x, 8.0F);

    // ---- Tiến từng khung theo tốc độ ----
    if (rs.playing && !rs.player.isFinished()) {
        rs.frame_accum += static_cast<double>(ImGui::GetIO().DeltaTime) * rs.speed * frames_per_second;
        const size_t n = static_cast<size_t>(rs.frame_accum);
        if (n > 0) {
            rs.frame_accum -= static_cast<double>(n);
            const size_t before = rs.pipeline.getTotalSteps();
            rs.player.step(n);
            rs.total_steps = rs.pipeline.getTotalSteps();
            (void)before;
        }
    }

    // ---- Kết quả ----
    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("RESULT", "KẾT QUẢ PIPELINE");
    smallLabel(rs.folder_name.c_str(), kDim);
    const auto pos = rs.pipeline.getDeadReckoningPosition();
    const double net = std::sqrt(pos.easting_m * pos.easting_m + pos.northing_m * pos.northing_m);
    char buf[96];
    if (ImGui::BeginTable("##replay_readouts", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "%04zu", rs.total_steps);
        readout("STEPS · BƯỚC", buf, nullptr, kText);
        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "%.2f", net);
        readout("LOOP 2D · ĐÓNG VÒNG", buf, "m", kAmber);
        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "%.0f", rs.latest.clusters[0].ce95_semi_major_m);
        readout("CE95", buf, "m", kText);
        ImGui::TableNextColumn();
        snprintf(buf, sizeof(buf), "%u", rs.latest.active_clusters_count);
        readout("CLUSTERS · CỤM", buf, nullptr, kText);
        ImGui::EndTable();
    }
    {
        const auto act = static_cast<uint8_t>(rs.latest.motion_activity);
        snprintf(buf, sizeof(buf), "%s · %s  (conf %.0f%%)", activityCode(act), activityVi(act),
                 rs.latest.activity_confidence * 100.0F);
        kvRow("ACTIVITY", buf);
        snprintf(buf, sizeof(buf), "%zu / %zu", rs.player.getCurrentFrameIndex(), rs.total_frames);
        kvRow("FRAME", buf);
    }

    // ---- Timeline hoạt động (tô màu 1 ô = 1s) ----
    ImGui::Dummy(ImVec2(0, 4));
    sectionHeader("ACTIVITY TIMELINE  1 ô = 1s", "DÒNG THỜI GIAN");
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 tl_origin = ImGui::GetCursorScreenPos();
    const float tl_w = ImGui::GetContentRegionAvail().x;
    const float tl_h = 22.0F;
    dl->AddRectFilled(tl_origin, ImVec2(tl_origin.x + tl_w, tl_origin.y + tl_h), kBg);
    double t_max = 1.0;
    for (const auto& [t, act] : rs.activity_timeline) {
        t_max = std::max(t_max, t);
    }
    for (const auto& [t, act] : rs.activity_timeline) {
        const float x0 = tl_origin.x + static_cast<float>(t / t_max) * tl_w;
        const float x1 = tl_origin.x + static_cast<float>((t + 1.0) / t_max) * tl_w;
        dl->AddRectFilled(ImVec2(x0, tl_origin.y + 3.0F),
                          ImVec2(std::max(x1 - 1.0F, x0 + 1.0F), tl_origin.y + tl_h - 3.0F), activityColor(act));
    }
    dl->AddRect(tl_origin, ImVec2(tl_origin.x + tl_w, tl_origin.y + tl_h), kLine);
    // Vị trí hiện tại
    if (rs.total_frames > 0) {
        // t hiện tại ≈ timestamp khung cuối - timestamp khung đầu
        const auto& f0 = rs.dataset.sensor_frames.front();
        const auto& fn = rs.dataset.sensor_frames
            [std::min(rs.player.getCurrentFrameIndex(), rs.total_frames - 1)];
        const double t_now = static_cast<double>(fn.timestamp_ns - f0.timestamp_ns) * 1e-9;
        const float cx = tl_origin.x + static_cast<float>(std::clamp(t_now / t_max, 0.0, 1.0)) * tl_w;
        dl->AddLine(ImVec2(cx, tl_origin.y - 3.0F), ImVec2(cx, tl_origin.y + tl_h + 3.0F), kText, 1.5F);
    }
    ImGui::Dummy(ImVec2(tl_w, tl_h + 4));

    // Chú giải (ô màu + nhãn)
    pushFont(fonts().label);
    for (uint8_t a : {uint8_t{0}, uint8_t{1}, uint8_t{2}, uint8_t{3}, uint8_t{4}, uint8_t{5}}) {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const float h = ImGui::GetTextLineHeight();
        dl->AddRectFilled(ImVec2(p.x, p.y + h * 0.5F - 4.0F), ImVec2(p.x + 8.0F, p.y + h * 0.5F + 4.0F),
                          activityColor(a));
        ImGui::Dummy(ImVec2(10.0F, h));
        ImGui::SameLine(0.0F, 4.0F);
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kMuted));
        ImGui::TextUnformatted(activityVi(a));
        ImGui::PopStyleColor();
        ImGui::SameLine(0.0F, 14.0F);
    }
    ImGui::NewLine();
    ImGui::PopFont();

    ImGui::End();
}

} // namespace desktop
