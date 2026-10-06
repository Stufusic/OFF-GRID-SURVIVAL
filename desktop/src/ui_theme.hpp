// ============================================================================
// UI THEME — "Tactical Instrument" design system (dùng chung mọi panel ImGui)
// Than chì + hổ phách/olive, font mono, viền mảnh góc vuông, dấu ngoặc góc.
// Palette khớp 1-1 với desktop/web/style.css (--bg, --amber, --olive, ...).
// ============================================================================
#pragma once

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace desktop::theme {

// Thay IM_COL32 (macro dùng cast kiểu C -> cảnh báo -Wold-style-cast)
[[nodiscard]] constexpr ImU32 rgba(unsigned r, unsigned g, unsigned b, unsigned a = 255U) noexcept {
    return (a << IM_COL32_A_SHIFT) | (b << IM_COL32_B_SHIFT) | (g << IM_COL32_G_SHIFT) | (r << IM_COL32_R_SHIFT);
}

// ---- Palette ---------------------------------------------------------------
inline constexpr ImU32 kBg          = rgba(14, 16, 15, 255);    // #0E100F
inline constexpr ImU32 kPanel       = rgba(19, 21, 19, 255);    // #131513
inline constexpr ImU32 kFrame       = rgba(28, 31, 28, 255);    // #1C1F1C
inline constexpr ImU32 kLine        = rgba(44, 48, 42, 255);    // #2C302A
inline constexpr ImU32 kLineStrong  = rgba(74, 80, 70, 255);    // #4A5046
inline constexpr ImU32 kGrid        = rgba(38, 42, 37, 255);    // lưới bản đồ
inline constexpr ImU32 kText        = rgba(217, 214, 204, 255); // #D9D6CC
inline constexpr ImU32 kMuted       = rgba(125, 128, 117, 255); // #7D8075
inline constexpr ImU32 kDim         = rgba(84, 88, 78, 255);    // #54584E
inline constexpr ImU32 kAmber       = rgba(232, 163, 61, 255);  // #E8A33D
inline constexpr ImU32 kAmberDim    = rgba(138, 100, 40, 255);  // #8A6428
inline constexpr ImU32 kOlive       = rgba(138, 154, 91, 255);  // #8A9A5B
inline constexpr ImU32 kOliveBright = rgba(176, 194, 118, 255); // #B0C276
inline constexpr ImU32 kRed         = rgba(217, 73, 58, 255);   // #D9493A
inline constexpr ImU32 kYellow      = rgba(232, 208, 61, 255);  // #E8D03D
inline constexpr ImU32 kBlue        = rgba(79, 143, 214, 255);  // #4F8FD6 — đường đồng mức
inline constexpr ImU32 kBlueSoft    = rgba(120, 170, 225, 255); // #78AAE1

[[nodiscard]] inline ImU32 withAlpha(ImU32 c, int a) noexcept {
    return (c & ~IM_COL32_A_MASK) | (static_cast<ImU32>(std::clamp(a, 0, 255)) << IM_COL32_A_SHIFT);
}
[[nodiscard]] inline ImVec4 v4(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }

// ---- Font set (nạp trong app_main.cpp::loadFonts) --------------------------
struct FontSet {
    ImFont* body{nullptr};   // 15px mono — văn bản chung
    ImFont* label{nullptr};  // 12px mono — nhãn viết hoa nhỏ
    ImFont* mid{nullptr};    // 19px mono — tiêu đề trạng thái
    ImFont* big{nullptr};    // 28px mono — số đo lớn
};
inline FontSet& fonts() {
    static FontSet f{};
    return f;
}
inline void pushFont(ImFont* f) { ImGui::PushFont(f != nullptr ? f : ImGui::GetFont()); }

// ---- Dấu ngoặc góc (corner brackets) kiểu ống ngắm --------------------------
inline void drawBrackets(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float len = 10.0F,
                         float th = 1.0F) {
    dl->AddLine(ImVec2(a.x, a.y), ImVec2(a.x + len, a.y), col, th);
    dl->AddLine(ImVec2(a.x, a.y), ImVec2(a.x, a.y + len), col, th);
    dl->AddLine(ImVec2(b.x, a.y), ImVec2(b.x - len, a.y), col, th);
    dl->AddLine(ImVec2(b.x - 1, a.y), ImVec2(b.x - 1, a.y + len), col, th);
    dl->AddLine(ImVec2(a.x, b.y - 1), ImVec2(a.x + len, b.y - 1), col, th);
    dl->AddLine(ImVec2(a.x, b.y), ImVec2(a.x, b.y - len), col, th);
    dl->AddLine(ImVec2(b.x, b.y - 1), ImVec2(b.x - len, b.y - 1), col, th);
    dl->AddLine(ImVec2(b.x - 1, b.y), ImVec2(b.x - 1, b.y - len), col, th);
}

// Vẽ ngoặc 4 góc của cửa sổ hiện tại (gọi ngay sau ImGui::Begin)
inline void windowBrackets() {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetWindowPos();
    const ImVec2 s = ImGui::GetWindowSize();
    const float title = ImGui::GetFrameHeight();  // vùng tab/title
    const ImVec2 a(p.x + 3.0F, p.y + title + 3.0F);
    const ImVec2 b(p.x + s.x - 3.0F, p.y + s.y - 3.0F);
    dl->PushClipRect(p, ImVec2(p.x + s.x, p.y + s.y), false);
    drawBrackets(dl, a, b, kLineStrong, 9.0F, 1.0F);
    dl->PopClipRect();
}

// ---- Tiêu đề mục: "CODE // tiếng Việt" + đường kẻ mảnh ----------------------
inline void sectionHeader(const char* code, const char* vi = nullptr) {
    pushFont(fonts().label);
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kText));
    ImGui::TextUnformatted(code);
    ImGui::PopStyleColor();
    if (vi != nullptr) {
        ImGui::SameLine(0.0F, 6.0F);
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kDim));
        ImGui::Text("// %s", vi);
        ImGui::PopStyleColor();
    }
    ImGui::PopFont();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::GetWindowDrawList()->AddLine(ImVec2(p.x, p.y + 1.0F), ImVec2(p.x + w, p.y + 1.0F), kLine);
    ImGui::Dummy(ImVec2(0.0F, 5.0F));
}

// Nhãn nhỏ viết hoa (muted)
inline void smallLabel(const char* text, ImU32 col = kMuted) {
    pushFont(fonts().label);
    ImGui::PushStyleColor(ImGuiCol_Text, v4(col));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

// ---- Số đo lớn: nhãn nhỏ phía trên, số mono lớn, đơn vị dim -----------------
inline void readout(const char* label, const char* value, const char* unit, ImU32 col) {
    ImGui::BeginGroup();
    smallLabel(label);
    pushFont(fonts().big);
    ImGui::PushStyleColor(ImGuiCol_Text, v4(col));
    ImGui::TextUnformatted(value);
    ImGui::PopStyleColor();
    ImGui::PopFont();
    if (unit != nullptr && unit[0] != '\0') {
        ImGui::SameLine(0.0F, 4.0F);
        const float big_h = fonts().big != nullptr ? fonts().big->FontSize : 28.0F;
        const float lbl_h = fonts().label != nullptr ? fonts().label->FontSize : 12.0F;
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (big_h - lbl_h) - 4.0F);
        smallLabel(unit, kDim);
    }
    ImGui::EndGroup();
}

// ---- Vạch tín hiệu (5 cột tăng dần) -----------------------------------------
inline void signalBars(float v01, ImU32 on, float h = 16.0F) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    constexpr int kN = 6;
    const float bw = 3.0F;
    const float gap = 2.0F;
    const int lit = static_cast<int>(std::round(std::clamp(v01, 0.0F, 1.0F) * kN));
    for (int i = 0; i < kN; ++i) {
        const float bh = h * (0.30F + 0.70F * static_cast<float>(i + 1) / kN);
        const float x = p.x + static_cast<float>(i) * (bw + gap);
        dl->AddRectFilled(ImVec2(x, p.y + h - bh), ImVec2(x + bw, p.y + h), i < lit ? on : kFrame);
    }
    ImGui::Dummy(ImVec2(kN * (bw + gap), h));
}

// ---- Thước phân đoạn (N vạch) — độ tin cậy, tiến trình ----------------------
inline void segmentedMeter(float v01, int n, ImU32 on, float w, float h = 12.0F) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float gap = 3.0F;
    w = std::max(w, static_cast<float>(n) * 4.0F);
    const float sw = (w - gap * static_cast<float>(n - 1)) / static_cast<float>(n);
    const int lit = static_cast<int>(std::round(std::clamp(v01, 0.0F, 1.0F) * static_cast<float>(n)));
    for (int i = 0; i < n; ++i) {
        const float x = p.x + static_cast<float>(i) * (sw + gap);
        dl->AddRectFilled(ImVec2(x, p.y), ImVec2(x + sw, p.y + h), i < lit ? on : kFrame);
    }
    ImGui::Dummy(ImVec2(w, h));
}

// ---- Ô trạng thái (ma trận sức khỏe) ---------------------------------------
enum class Cell { kOk, kInfoOn, kInfoOff, kFault };

inline void statusCell(const char* label, Cell state, ImVec2 size) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImU32 bg = kFrame;
    ImU32 border = kLine;
    ImU32 fg = kDim;
    switch (state) {
        case Cell::kOk: bg = rgba(46, 54, 32, 255); border = kOlive; fg = kOliveBright; break;
        case Cell::kInfoOn: bg = rgba(26, 42, 62, 255); border = kBlue; fg = kBlueSoft; break;
        case Cell::kInfoOff: break;
        case Cell::kFault: {
            const float pulse = 0.65F + 0.35F * std::sin(static_cast<float>(ImGui::GetTime()) * 6.0F);
            bg = withAlpha(rgba(120, 36, 28, 255), static_cast<int>(255 * pulse));
            border = kRed;
            fg = rgba(255, 205, 195, 255);
            break;
        }
    }
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), bg);
    dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), border);
    pushFont(fonts().label);
    const ImVec2 ts = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(p.x + (size.x - ts.x) * 0.5F, p.y + (size.y - ts.y) * 0.5F), fg, label);
    ImGui::PopFont();
    ImGui::Dummy(size);
}

// ---- Dải trạng thái an toàn toàn chiều rộng ---------------------------------
inline void statusStrip(const char* text, ImU32 col, bool pulse, float h = 30.0F) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    float a = 1.0F;
    if (pulse) a = 0.55F + 0.45F * std::sin(static_cast<float>(ImGui::GetTime()) * 5.0F);
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), withAlpha(col, static_cast<int>(40 * a + 14)));
    dl->AddRectFilled(p, ImVec2(p.x + 3.0F, p.y + h), col);
    const float sq = 8.0F;
    const ImVec2 sp(p.x + 14.0F, p.y + (h - sq) * 0.5F);
    dl->AddRectFilled(sp, ImVec2(sp.x + sq, sp.y + sq), withAlpha(col, static_cast<int>(255 * a)));
    pushFont(fonts().body);
    const ImVec2 ts = ImGui::CalcTextSize(text);
    dl->AddText(ImVec2(sp.x + sq + 10.0F, p.y + (h - ts.y) * 0.5F), col, text);
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(w, h));
}

// ---- Bảng key/value mono (bằng chứng cảm biến, thông số) -------------------
inline void kvRow(const char* key, const char* fmt_value) {
    pushFont(fonts().label);
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kMuted));
    ImGui::TextUnformatted(key);
    ImGui::PopStyleColor();
    ImGui::PopFont();
    ImGui::SameLine(150.0F);
    ImGui::TextUnformatted(fmt_value);
}

// ---- Hoạt động: mã ngắn (EN) + nhãn tiếng Việt có dấu + màu thống nhất -----
// (khớp MotionActivity enum trong types.hpp; hud_math::activityLabel giữ nguyên cho test)
[[nodiscard]] inline const char* activityCode(uint8_t act) noexcept {
    static const char* k[] = {"IDLE", "WALK", "RUN", "STAIR UP", "STAIR DN", "LANDING",
                              "ESCALATOR", "CYCLE", "VEHICLE", "ELEVATOR", "CABLE CAR"};
    return act < 11 ? k[act] : "UNKNOWN";
}
[[nodiscard]] inline const char* activityVi(uint8_t act) noexcept {
    static const char* k[] = {"ĐỨNG YÊN", "ĐI BỘ", "CHẠY", "LÊN THANG", "XUỐNG THANG", "CHIẾU NGHỈ",
                              "THANG CUỐN", "ĐẠP XE", "XE CƠ", "THANG MÁY", "CÁP TREO"};
    return act < 11 ? k[act] : "KHÔNG RÕ";
}
[[nodiscard]] inline ImU32 activityColor(uint8_t act) noexcept {
    switch (act) {
        case 0: return kDim;                            // đứng yên
        case 1: return kAmber;                          // đi bộ
        case 2: return kYellow;                         // chạy
        case 3: return kOliveBright;                    // lên thang
        case 4: return rgba(204, 112, 64, 255);     // xuống thang (đất nung)
        case 5: return kOlive;                          // chiếu nghỉ
        default: return kMuted;
    }
}

} // namespace desktop::theme
