// ============================================================================
// MAP PANEL — bản đồ tactical với zoom/pan chuột, 4 cụm giả thuyết
// Phong cách "Tactical Instrument": lưới UTM thích ứng có nhãn tọa độ mép,
// đường đồng mức XANH DƯƠNG, đám hạt hổ phách, tâm ngắm vị trí ước lượng,
// mũi tên Bắc + thước tỉ lệ, nút zoom dạng [+] [−] [FIT] phủ trên bản đồ.
// ============================================================================
#include "imgui.h"
#include "ui_panels.hpp"
#include "ui_theme.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace desktop {

namespace {

using namespace theme;

constexpr double kPi = 3.14159265358979323846;

const ImU32 kClusterColors[4] = {
    kAmber,                       // H0 — cụm chính
    kOliveBright,                 // H1
    kText,                        // H2
    rgba(204, 112, 64, 255),  // H3 — đất nung
};

// Chọn bước "đẹp" nhỏ nhất sao cho step * ppm >= min_px
double niceStep(double ppm, double min_px) {
    static const double kSteps[] = {1, 2, 5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000};
    for (double s : kSteps) {
        if (s * ppm >= min_px) return s;
    }
    return 10000.0;
}

} // namespace

void renderMapPanel(AppState& app, MapView& view) {
    ImGui::SetNextWindowSizeConstraints(ImVec2(360, 300), ImVec2(FLT_MAX, FLT_MAX));
    if (!ImGui::Begin(kWinMap)) {
        ImGui::End();
        return;
    }
    windowBrackets();

    const float footer_h = ImGui::GetTextLineHeightWithSpacing() + 8.0F;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const double vp_w = std::max(80.0, static_cast<double>(avail.x));
    const double vp_h = std::max(80.0, static_cast<double>(avail.y - footer_h));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 end(origin.x + static_cast<float>(vp_w), origin.y + static_cast<float>(vp_h));

    // Tương tác: zoom scroll, pan kéo chuột (cho phép nút overlay đè lên)
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("map_canvas", ImVec2(static_cast<float>(vp_w), static_cast<float>(vp_h)),
                           ImGuiButtonFlags_MouseButtonLeft);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const ImVec2 after_canvas = ImGui::GetCursorScreenPos();
    if (hovered && ImGui::GetIO().MouseWheel != 0.0F) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        view.zoomAt(mouse.x - origin.x, mouse.y - origin.y, vp_w, vp_h,
                    ImGui::GetIO().MouseWheel > 0.0F ? 1.15 : 1.0 / 1.15);
    }
    if (active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        const ImVec2 drag = ImGui::GetIO().MouseDelta;
        view.panPixels(drag.x, drag.y);
    }

    // world -> screen tuyệt đối (MapView trả tọa độ tương đối viewport)
    auto SX = [&](double e) { return origin.x + view.screenX(e, vp_w); };
    auto SY = [&](double n) { return origin.y + view.screenY(n, vp_h); };

    dl->PushClipRect(origin, end, true);
    dl->AddRectFilled(origin, end, kBg);

    // ---- Lưới UTM thích ứng + nhãn tọa độ mép ----
    const double e_min = view.worldEast(0, vp_w);
    const double e_max = view.worldEast(vp_w, vp_w);
    const double n_min = view.worldNorth(vp_h, vp_h);
    const double n_max = view.worldNorth(0, vp_h);
    const double grid_m = niceStep(view.pixels_per_m, 90.0);
    const double minor_m = grid_m / 5.0;
    if (minor_m * view.pixels_per_m >= 12.0) {
        for (double e = std::floor(e_min / minor_m) * minor_m; e <= e_max; e += minor_m) {
            const float x = SX(e);
            for (double n = std::floor(n_min / minor_m) * minor_m; n <= n_max; n += minor_m) {
                dl->AddRectFilled(ImVec2(x, SY(n)), ImVec2(x + 1.0F, SY(n) + 1.0F), kLine);  // lưới chấm
            }
        }
    }
    pushFont(fonts().label);
    char lbl[32];
    for (double e = std::floor(e_min / grid_m) * grid_m; e <= e_max; e += grid_m) {
        const float x = SX(e);
        dl->AddLine(ImVec2(x, origin.y), ImVec2(x, end.y), kGrid, 1.0F);
        snprintf(lbl, sizeof(lbl), "E %04.0f", e);
        dl->AddText(ImVec2(x + 4.0F, origin.y + 22.0F), kMuted, lbl);
    }
    for (double n = std::floor(n_min / grid_m) * grid_m; n <= n_max; n += grid_m) {
        const float y = SY(n);
        dl->AddLine(ImVec2(origin.x, y), ImVec2(end.x, y), kGrid, 1.0F);
        snprintf(lbl, sizeof(lbl), "N %04.0f", n);
        dl->AddText(ImVec2(origin.x + 6.0F, y - 15.0F), kMuted, lbl);
    }
    ImGui::PopFont();

    // ---- Đường đồng mức (xanh dương) — núi cone đỉnh (1000,1000), chân r=500m ----
    for (int k = 1; k <= 10; ++k) {
        const double r = 50.0 * k;
        const bool index_line = (k % 2 == 0);  // đường cái đậm hơn
        ImVector<ImVec2> ring;
        ring.resize(96);
        for (int i = 0; i < 96; ++i) {
            const double t = i * 2.0 * kPi / 96.0;
            ring[i] = ImVec2(SX(1000.0 + r * std::cos(t)), SY(1000.0 + r * std::sin(t)));
        }
        dl->AddPolyline(ring.Data, ring.Size, withAlpha(kBlue, index_line ? 170 : 80),
                        ImDrawFlags_Closed, index_line ? 1.3F : 1.0F);
    }
    {
        const float px = SX(1000.0);
        const float py = SY(1000.0);
        dl->AddTriangleFilled(ImVec2(px, py - 6.0F), ImVec2(px - 5.0F, py + 4.0F), ImVec2(px + 5.0F, py + 4.0F),
                              kBlueSoft);
        pushFont(fonts().label);
        dl->AddText(ImVec2(px + 8.0F, py - 6.0F), kBlueSoft, "PK");
        ImGui::PopFont();
    }

    // ---- Trail (đường đi thật, cap 500 điểm) ----
    if (app.trail_history.size() >= 2) {
        ImVector<ImVec2> pts;
        pts.reserve(static_cast<int>(app.trail_history.size()));
        for (const auto& [e, n] : app.trail_history) pts.push_back(ImVec2(SX(e), SY(n)));
        dl->AddPolyline(pts.Data, pts.Size, withAlpha(kAmber, 210), 0, 1.6F);
    }

    // ---- PF cloud ----
    const auto& particles = app.pipeline.getParticleFilter().getParticles();
    const ImU32 pf_col = withAlpha(kAmber, 120);
    for (const auto& p : particles) {
        if (!p.is_alive) continue;
        const float px = SX(p.easting_m);
        const float py = SY(p.northing_m);
        if (px < origin.x - 4 || px > end.x + 4 || py < origin.y - 4 || py > end.y + 4) continue;
        dl->AddRectFilled(ImVec2(px - 1.0F, py - 1.0F), ImVec2(px + 1.0F, py + 1.0F), pf_col);
    }

    // ---- Cụm giả thuyết (tối đa 4) + ellipse CE95 ----
    pushFont(fonts().label);
    const uint8_t n_clusters = std::min<uint8_t>(app.latest_state.active_clusters_count, 4);
    for (uint8_t ci = 0; ci < n_clusters; ++ci) {
        const auto& c = app.latest_state.clusters[ci];
        if (c.ce95_semi_major_m <= 0.0) continue;
        const auto outline = ellipseOutline(c.center_easting_m, c.center_northing_m, c.ce95_semi_major_m,
                                            c.ce95_semi_minor_m, c.ellipse_heading_rad);
        ImVec2 pts[64];
        for (size_t i = 0; i < 64; ++i) pts[i] = ImVec2(SX(outline[i].first), SY(outline[i].second));
        const ImU32 col = kClusterColors[ci];
        dl->AddPolyline(pts, 64, withAlpha(col, 220), ImDrawFlags_Closed, 1.3F);
        const float cx = SX(c.center_easting_m);
        const float cy = SY(c.center_northing_m);
        if (ci > 0) {
            dl->AddLine(ImVec2(cx - 4, cy), ImVec2(cx + 4, cy), col, 1.0F);
            dl->AddLine(ImVec2(cx, cy - 4), ImVec2(cx, cy + 4), col, 1.0F);
        }
        snprintf(lbl, sizeof(lbl), "H%u", ci);
        // nhãn đặt ở đầu bán trục lớn
        const ImVec2 tip = pts[0];
        dl->AddText(ImVec2(tip.x + 6.0F, tip.y - 14.0F), col, lbl);
    }
    ImGui::PopFont();

    // ---- Tâm ngắm vị trí ước lượng (cụm H0) ----
    if (n_clusters > 0) {
        const auto& c = app.latest_state.clusters[0];
        const float cx = SX(c.center_easting_m);
        const float cy = SY(c.center_northing_m);
        const float g = 4.0F;
        const float L = 13.0F;
        dl->AddLine(ImVec2(cx - L, cy), ImVec2(cx - g, cy), kAmber, 1.5F);
        dl->AddLine(ImVec2(cx + g, cy), ImVec2(cx + L, cy), kAmber, 1.5F);
        dl->AddLine(ImVec2(cx, cy - L), ImVec2(cx, cy - g), kAmber, 1.5F);
        dl->AddLine(ImVec2(cx, cy + g), ImVec2(cx, cy + L), kAmber, 1.5F);
        drawBrackets(dl, ImVec2(cx - 9, cy - 9), ImVec2(cx + 10, cy + 10), withAlpha(kAmber, 160), 4.0F, 1.0F);
    }

    // ---- Vị trí thật: thoi rỗng + vạch hướng ----
    {
        const float px = SX(app.true_easting_m);
        const float py = SY(app.true_northing_m);
        const float r = 5.5F;
        dl->AddQuad(ImVec2(px, py - r), ImVec2(px + r, py), ImVec2(px, py + r), ImVec2(px - r, py), kText, 1.5F);
        const float hx = px + 22.0F * static_cast<float>(std::sin(app.true_heading_rad));
        const float hy = py - 22.0F * static_cast<float>(std::cos(app.true_heading_rad));
        dl->AddLine(ImVec2(px, py), ImVec2(hx, hy), kText, 1.5F);
    }

    // ---- Tiêu đề overlay góc trên trái ----
    pushFont(fonts().label);
    snprintf(lbl, sizeof(lbl), "UTM GRID %.0f m", grid_m);
    dl->AddRectFilled(ImVec2(origin.x, origin.y), ImVec2(origin.x + 150.0F, origin.y + 18.0F), withAlpha(kBg, 220));
    dl->AddText(ImVec2(origin.x + 6.0F, origin.y + 3.0F), kText, lbl);

    // ---- Mũi tên Bắc + thước tỉ lệ (góc dưới trái) ----
    {
        const float bx = origin.x + 22.0F;
        const float by = end.y - 22.0F;
        dl->AddTriangleFilled(ImVec2(bx, by - 26.0F), ImVec2(bx - 6.0F, by - 12.0F), ImVec2(bx + 6.0F, by - 12.0F),
                              kText);
        dl->AddText(ImVec2(bx - 3.5F, by - 9.0F), kText, "N");
        const double scale_m = niceStep(view.pixels_per_m, 70.0);
        const float sw = static_cast<float>(scale_m * view.pixels_per_m);
        const float sx0 = bx + 22.0F;
        dl->AddLine(ImVec2(sx0, by), ImVec2(sx0 + sw, by), kText, 1.5F);
        dl->AddLine(ImVec2(sx0, by - 5), ImVec2(sx0, by + 1), kText, 1.5F);
        dl->AddLine(ImVec2(sx0 + sw, by - 5), ImVec2(sx0 + sw, by + 1), kText, 1.5F);
        dl->AddLine(ImVec2(sx0 + sw * 0.5F, by - 3), ImVec2(sx0 + sw * 0.5F, by), kText, 1.0F);
        snprintf(lbl, sizeof(lbl), "%.0f m", scale_m);
        dl->AddText(ImVec2(sx0, by - 18.0F), kText, lbl);
    }
    ImGui::PopFont();

    // ---- Nút zoom overlay (góc dưới phải) ----
    {
        ImGui::PushStyleColor(ImGuiCol_Button, v4(withAlpha(kBg, 230)));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 3));
        const float bw_plus = ImGui::CalcTextSize("[+]").x + 16.0F;
        const float bw_fit = ImGui::CalcTextSize("[FIT]").x + 16.0F;
        const float total = bw_plus * 2.0F + bw_fit + 12.0F;
        ImGui::SetCursorScreenPos(ImVec2(end.x - total - 12.0F, end.y - 34.0F));
        if (ImGui::Button("[+]")) view.zoomAt(vp_w * 0.5, vp_h * 0.5, vp_w, vp_h, 1.25);
        ImGui::SameLine(0.0F, 6.0F);
        if (ImGui::Button("[−]")) view.zoomAt(vp_w * 0.5, vp_h * 0.5, vp_w, vp_h, 1.0 / 1.25);
        ImGui::SameLine(0.0F, 6.0F);
        if (ImGui::Button("[FIT]")) {
            auto pts = app.trail_history;
            pts.emplace_back(app.true_easting_m, app.true_northing_m);
            view.fitTo(pts, vp_w, vp_h, 60.0);
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }

    dl->PopClipRect();
    dl->AddRect(origin, end, kLine);

    // ---- Footer: chú giải + tọa độ tâm ----
    ImGui::SetCursorScreenPos(ImVec2(after_canvas.x, after_canvas.y + 4.0F));
    pushFont(fonts().label);
    auto legend = [](const char* glyph, ImU32 col, const char* text) {
        ImGui::PushStyleColor(ImGuiCol_Text, v4(col));
        ImGui::TextUnformatted(glyph);
        ImGui::PopStyleColor();
        ImGui::SameLine(0.0F, 4.0F);
        ImGui::PushStyleColor(ImGuiCol_Text, v4(kMuted));
        ImGui::TextUnformatted(text);
        ImGui::PopStyleColor();
        ImGui::SameLine(0.0F, 16.0F);
    };
    legend("+", kAmber, "ƯỚC LƯỢNG");
    legend("◇", kText, "VỊ TRÍ THẬT");
    legend("·", kAmber, "PF CLOUD");
    legend("◯", kOliveBright, "CE95 H0–H3");
    legend("~", kBlue, "ĐỒNG MỨC");
    char info[96];
    snprintf(info, sizeof(info), "E %07.1f  N %07.1f  ·  %.2f px/m", view.center_east_m, view.center_north_m,
             view.pixels_per_m);
    const float iw = ImGui::CalcTextSize(info).x;
    ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - iw));
    ImGui::PushStyleColor(ImGuiCol_Text, v4(kDim));
    ImGui::TextUnformatted(info);
    ImGui::PopStyleColor();
    ImGui::PopFont();

    ImGui::End();
}

} // namespace desktop
