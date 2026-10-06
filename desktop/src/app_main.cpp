// ============================================================================
// APP MAIN — Dear ImGui + backend Win32 + DirectX11
// Dashboard OFF-GRID SURVIVAL: bản đồ tactical + telemetry + safety + rescue
// + controls + replay dữ liệu thật. Thay thế monolith Win32/GDI+ cũ.
// ============================================================================
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef UNICODE
#define UNICODE
#endif
#include <windows.h>

#include <d3d11.h>
#include <dxgi.h>
#include <tchar.h>

#include "imgui.h"
#include "imgui_internal.h"  // DockBuilder API
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"

#include "app_state.hpp"
#include "ui_panels.hpp"
#include "ui_theme.hpp"
#include "web_server.hpp"

#include <cmath>
#include <filesystem>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg,
                                                             WPARAM wParam, LPARAM lParam);

namespace {

// ---- D3D prototypes ----
bool CreateRenderTarget();

// ---- D3D boilerplate (chuẩn ImGui sample Win32+DX11) ----
ID3D11Device* g_pd3dDevice = nullptr;
ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
IDXGISwapChain* g_pSwapChain = nullptr;
ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;
HWND g_hwnd = nullptr;

bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd{};
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT create_flags = 0;
#ifdef NDEBUG
    // Release: vẫn giữ debug layer tắt
#else
    create_flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
    D3D_FEATURE_LEVEL feature_level;
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1,
                                        D3D_FEATURE_LEVEL_10_0};
    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, create_flags, levels,
        sizeof(levels) / sizeof(levels[0]), D3D11_SDK_VERSION, &sd, &g_pSwapChain,
        &g_pd3dDevice, &feature_level, &g_pd3dDeviceContext);
    if (FAILED(hr)) {
        // Fallback WARP (software rasterizer) — chạy được cả trên VM
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels,
            sizeof(levels) / sizeof(levels[0]), D3D11_SDK_VERSION, &sd, &g_pSwapChain,
            &g_pd3dDevice, &feature_level, &g_pd3dDeviceContext);
        if (FAILED(hr)) return false;
    }
    return CreateRenderTarget();
}

void CleanupDeviceD3D() {
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

bool CreateRenderTarget() {
    ID3D11Texture2D* back_buffer = nullptr;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&back_buffer));
    if (back_buffer == nullptr) return false;
    const HRESULT hr = g_pd3dDevice->CreateRenderTargetView(back_buffer, nullptr, &g_mainRenderTargetView);
    back_buffer->Release();
    return SUCCEEDED(hr);
}

void ResizeTarget(WORD w, WORD h) {
    if (g_pSwapChain == nullptr) return;
    g_pd3dDeviceContext->OMSetRenderTargets(0, nullptr, nullptr);
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
    g_pSwapChain->ResizeBuffers(0, w, h, DXGI_FORMAT_UNKNOWN, 0);
    CreateRenderTarget();
}

// ---- App state ----
desktop::AppState g_app{};
desktop::MapView g_map_view{};

// ---- Theme "Tactical Instrument": than chì + hổ phách/olive, góc vuông ----
void applyTacticalTheme() {
    using namespace desktop::theme;
    ImGuiStyle& st = ImGui::GetStyle();
    st.WindowRounding = 0.0F;
    st.ChildRounding = 0.0F;
    st.FrameRounding = 0.0F;
    st.GrabRounding = 0.0F;
    st.PopupRounding = 0.0F;
    st.ScrollbarRounding = 0.0F;
    st.TabRounding = 0.0F;
    st.WindowBorderSize = 1.0F;
    st.FrameBorderSize = 1.0F;
    st.ChildBorderSize = 1.0F;
    st.TabBorderSize = 0.0F;
    st.FramePadding = ImVec2(10, 5);
    st.ItemSpacing = ImVec2(10, 7);
    st.ItemInnerSpacing = ImVec2(6, 4);
    st.WindowPadding = ImVec2(14, 12);
    st.ScrollbarSize = 10.0F;
    st.GrabMinSize = 8.0F;
    st.DockingSeparatorSize = 1.0F;
    st.WindowTitleAlign = ImVec2(0.0F, 0.5F);

    ImVec4* c = st.Colors;
    c[ImGuiCol_Text] = v4(kText);
    c[ImGuiCol_TextDisabled] = v4(kMuted);
    c[ImGuiCol_WindowBg] = v4(kPanel);
    c[ImGuiCol_ChildBg] = v4(kBg);
    c[ImGuiCol_PopupBg] = v4(withAlpha(kPanel, 250));
    c[ImGuiCol_Border] = v4(kLine);
    c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_FrameBg] = v4(kBg);
    c[ImGuiCol_FrameBgHovered] = v4(kFrame);
    c[ImGuiCol_FrameBgActive] = v4(rgba(36, 40, 34, 255));
    c[ImGuiCol_TitleBg] = v4(kBg);
    c[ImGuiCol_TitleBgActive] = v4(kBg);
    c[ImGuiCol_TitleBgCollapsed] = v4(kBg);
    c[ImGuiCol_MenuBarBg] = v4(kBg);
    c[ImGuiCol_ScrollbarBg] = v4(kPanel);
    c[ImGuiCol_ScrollbarGrab] = v4(kLine);
    c[ImGuiCol_ScrollbarGrabHovered] = v4(kLineStrong);
    c[ImGuiCol_ScrollbarGrabActive] = v4(kAmberDim);
    c[ImGuiCol_CheckMark] = v4(kAmber);
    c[ImGuiCol_SliderGrab] = v4(kAmber);
    c[ImGuiCol_SliderGrabActive] = v4(rgba(255, 190, 90, 255));
    c[ImGuiCol_Button] = v4(kBg);
    c[ImGuiCol_ButtonHovered] = v4(rgba(40, 34, 22, 255));
    c[ImGuiCol_ButtonActive] = v4(kAmberDim);
    c[ImGuiCol_Header] = v4(kFrame);
    c[ImGuiCol_HeaderHovered] = v4(rgba(40, 34, 22, 255));
    c[ImGuiCol_HeaderActive] = v4(kAmberDim);
    c[ImGuiCol_Separator] = v4(kLine);
    c[ImGuiCol_SeparatorHovered] = v4(kAmberDim);
    c[ImGuiCol_SeparatorActive] = v4(kAmber);
    c[ImGuiCol_ResizeGrip] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ResizeGripHovered] = v4(kAmberDim);
    c[ImGuiCol_ResizeGripActive] = v4(kAmber);
    c[ImGuiCol_Tab] = v4(kBg);
    c[ImGuiCol_TabHovered] = v4(rgba(40, 34, 22, 255));
    c[ImGuiCol_TabActive] = v4(kPanel);
    c[ImGuiCol_TabUnfocused] = v4(kBg);
    c[ImGuiCol_TabUnfocusedActive] = v4(kPanel);
    c[ImGuiCol_DockingPreview] = v4(withAlpha(kAmber, 110));
    c[ImGuiCol_DockingEmptyBg] = v4(kBg);
    c[ImGuiCol_PlotLines] = v4(kAmber);
    c[ImGuiCol_PlotHistogram] = v4(kAmber);
    c[ImGuiCol_TableHeaderBg] = v4(kBg);
    c[ImGuiCol_TableBorderStrong] = v4(kLine);
    c[ImGuiCol_TableBorderLight] = v4(kLine);
    c[ImGuiCol_TextSelectedBg] = v4(withAlpha(kAmber, 70));
    c[ImGuiCol_NavHighlight] = v4(kAmber);

    // ImPlot: nền trong suốt, lưới mảnh, chữ muted — kiểu máy hiện sóng
    ImPlotStyle& ps = ImPlot::GetStyle();
    ps.PlotBorderSize = 1.0F;
    ps.LineWeight = 1.4F;
    ps.PlotPadding = ImVec2(6, 6);
    ps.LegendPadding = ImVec2(8, 6);
    ps.LegendInnerPadding = ImVec2(6, 3);
    ps.MinorAlpha = 0.5F;
    ImVec4* pc = ps.Colors;
    pc[ImPlotCol_FrameBg] = ImVec4(0, 0, 0, 0);
    pc[ImPlotCol_PlotBg] = v4(kBg);
    pc[ImPlotCol_PlotBorder] = v4(kLine);
    pc[ImPlotCol_LegendBg] = v4(withAlpha(kPanel, 235));
    pc[ImPlotCol_LegendBorder] = v4(kLine);
    pc[ImPlotCol_LegendText] = v4(kText);
    pc[ImPlotCol_TitleText] = v4(kText);
    pc[ImPlotCol_InlayText] = v4(kMuted);
    pc[ImPlotCol_AxisText] = v4(kMuted);
    pc[ImPlotCol_AxisGrid] = v4(withAlpha(kLine, 200));
    pc[ImPlotCol_AxisTick] = v4(kLine);
    pc[ImPlotCol_Crosshairs] = v4(kAmber);
}

// Font mono có glyph tiếng Việt: Cascadia Mono / Consolas (sẵn trên Win10/11).
// Có thể thả JetBrainsMono-Regular.ttf vào desktop/assets/fonts để ưu tiên.
void loadFonts(ImGuiIO& io) {
    const char* font_candidates[] = {
        "../assets/fonts/JetBrainsMono-Regular.ttf",
        "../../desktop/assets/fonts/JetBrainsMono-Regular.ttf",
        "C:/Windows/Fonts/CascadiaMono.ttf",
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
    };
    auto& fs = desktop::theme::fonts();
    // Tiếng Việt + ký hiệu tactical: — … → ↑ − ■ ▲ ◆ ● ◯
    static ImVector<ImWchar> ranges_vec;
    {
        ImFontGlyphRangesBuilder rb;
        rb.AddRanges(io.Fonts->GetGlyphRangesVietnamese());
        static const ImWchar extra[] = {0x0370, 0x03FF, 0x2010, 0x2027, 0x2070, 0x209F,
                                        0x2190, 0x21FF, 0x2200, 0x22FF, 0x2500, 0x25FF, 0};
        rb.AddRanges(extra);
        rb.BuildRanges(&ranges_vec);
    }
    const ImWchar* ranges = ranges_vec.Data;
    for (const char* path : font_candidates) {
        if (GetFileAttributesA(path) == INVALID_FILE_ATTRIBUTES) continue;
        fs.body = io.Fonts->AddFontFromFileTTF(path, 15.0F, nullptr, ranges);
        if (fs.body == nullptr) continue;
        fs.label = io.Fonts->AddFontFromFileTTF(path, 12.0F, nullptr, ranges);
        fs.mid = io.Fonts->AddFontFromFileTTF(path, 19.0F, nullptr, ranges);
        fs.big = io.Fonts->AddFontFromFileTTF(path, 28.0F, nullptr, ranges);
        break;
    }
    if (fs.body == nullptr) {
        fs.body = io.Fonts->AddFontDefault();  // fallback không dấu
        fs.label = fs.mid = fs.big = fs.body;
    }
    io.FontDefault = fs.body;
    io.Fonts->Build();
}

void handleShortcuts(const ImGuiIO& io) {
    if (io.WantTextInput) return;
    if (ImGui::IsKeyPressed(ImGuiKey_W)) g_app.simulateStep(0.75, 0.0);
    if (ImGui::IsKeyPressed(ImGuiKey_A)) g_app.simulateStep(0.75, -0.2618);
    if (ImGui::IsKeyPressed(ImGuiKey_D)) g_app.simulateStep(0.75, 0.2618);
    if (ImGui::IsKeyPressed(ImGuiKey_Space)) g_app.auto_walk = !g_app.auto_walk;
    if (ImGui::IsKeyPressed(ImGuiKey_F)) g_app.triggerFallEvent();
    if (ImGui::IsKeyPressed(ImGuiKey_C)) {
        g_app.safety_auditor.cancelAlert();
        g_app.latest_state.safety_state = g_app.safety_auditor.getState();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_G)) g_app.gps_enabled = !g_app.gps_enabled;
    if (ImGui::IsKeyPressed(ImGuiKey_B)) {
        g_app.battery_soc = (g_app.battery_soc > 10.0) ? (g_app.battery_soc - 20.0) : 95.0;
        g_app.resource_governor.updateBatterySoc(g_app.battery_soc);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_R)) g_app.initialize();
}

void renderDockSpace() {
    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_DockingEnable) {
        const ImGuiID dock_id = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        static bool first_frame = true;
        if (first_frame) {
            first_frame = false;
            ImGui::DockBuilderRemoveNode(dock_id);
            ImGui::DockBuilderAddNode(dock_id, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dock_id, ImGui::GetMainViewport()->WorkSize);
            ImGuiID dock_left = 0;
            ImGuiID dock_right = 0;
            ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Left, 0.58F, &dock_left, &dock_right);
            ImGuiID dock_right_up = 0;
            ImGuiID dock_right_down = 0;
            ImGui::DockBuilderSplitNode(dock_right, ImGuiDir_Up, 0.55F, &dock_right_up, &dock_right_down);
            ImGui::DockBuilderDockWindow(desktop::kWinMap, dock_left);
            ImGui::DockBuilderDockWindow(desktop::kWinReplay, dock_left);
            ImGui::DockBuilderDockWindow(desktop::kWinTelemetry, dock_right_up);
            ImGui::DockBuilderDockWindow(desktop::kWinControls, dock_right_up);
            ImGui::DockBuilderDockWindow(desktop::kWinSafety, dock_right_down);
            ImGui::DockBuilderDockWindow(desktop::kWinRescue, dock_right_down);
            ImGui::DockBuilderDockWindow(desktop::kWinWeb, dock_right_down);
            ImGui::DockBuilderFinish(dock_id);
        }
    }
}

} // namespace

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return 1;
    switch (msg) {
    case WM_SIZE:
        if (g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED) {
            ResizeTarget(LOWORD(lParam), HIWORD(lParam));
        }
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xFFF0) == SC_KEYMENU) return 0;  // chặn menu Alt
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        break;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR, int nCmdShow) {
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(WNDCLASSEXW);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.lpszClassName = L"OffGridSurvivalDashboard";
    RegisterClassExW(&wc);

    g_hwnd = CreateWindowExW(0, wc.lpszClassName,
                             L"OFF-GRID SURVIVAL ENGINE v2.3 - NAVIGATION DASHBOARD",
                             WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                             1440, 900, nullptr, nullptr, hInstance, nullptr);

    if (g_hwnd == nullptr || !CreateDeviceD3D(g_hwnd)) {
        CleanupDeviceD3D();
        return 1;
    }
    ShowWindow(g_hwnd, nCmdShow);
    UpdateWindow(g_hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImPlot::CreateContext();
    applyTacticalTheme();
    loadFonts(io);

    ImGui_ImplWin32_Init(g_hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    g_app.initialize();

    // [W3] --web-port N : auto-start web server khi khởi động (smoke test/tiện dụng)
    {
        int argc = 0;
        LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
        if (argv != nullptr) {
            for (int i = 1; i < argc - 1; ++i) {
                if (wcscmp(argv[i], L"--web-port") == 0) {
                    const int port = _wtoi(argv[i + 1]);
                    if (port >= 1024 && port <= 65535) {
                        const std::string web_dir = (std::filesystem::path("../web")).string();
                        desktop::g_web_started_ref() = desktop::g_web_server_ref().start(port, false, web_dir);
                    }
                    break;
                }
            }
            LocalFree(argv);
        }
    }

    bool running = true;
    while (running) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
            if (msg.message == WM_QUIT) running = false;
        }
        if (!running) break;

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // FPS thật đo từ vòng lặp
        g_app.fps_measured = io.Framerate;

        // Mô phỏng pacing theo wall-clock
        const double dt = std::clamp(static_cast<double>(io.DeltaTime), 0.001, 0.1);
        if (g_app.auto_walk) g_app.autoWalkTick(dt);

        handleShortcuts(io);

        // Fullscreen dockspace + panels
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        ImGui::Begin("##dockroot", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                         ImGuiWindowFlags_NoBackground);
        renderDockSpace();
        ImGui::End();
        ImGui::PopStyleVar(3);

        // Thanh trạng thái trên cùng: brand | đồng hồ nhiệm vụ | an toàn | link
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12, 8));
        if (ImGui::BeginMainMenuBar()) {
            using namespace desktop::theme;
            const float bar_w = ImGui::GetWindowWidth();
            ImGui::PushStyleColor(ImGuiCol_Text, v4(kAmber));
            ImGui::TextUnformatted("■");
            ImGui::PopStyleColor();
            ImGui::TextUnformatted("OFF-GRID SURVIVAL");
            ImGui::PushStyleColor(ImGuiCol_Text, v4(kDim));
            ImGui::TextUnformatted("// NAV-CORE v2.3");
            ImGui::PopStyleColor();

            // Đồng hồ nhiệm vụ T+ (thời gian mô phỏng)
            const uint64_t t0 = 1700000000000000000ULL;
            const uint64_t el = (g_app.current_time_ns > t0) ? (g_app.current_time_ns - t0) / 1000000000ULL : 0;
            char clock[32];
            snprintf(clock, sizeof(clock), "T+%02llu:%02llu:%02llu",
                     static_cast<unsigned long long>(el / 3600), static_cast<unsigned long long>((el / 60) % 60),
                     static_cast<unsigned long long>(el % 60));
            ImGui::SameLine(bar_w * 0.5F - ImGui::CalcTextSize(clock).x * 0.5F);
            ImGui::TextUnformatted(clock);

            // Chip an toàn + link + FPS + PWR (căn phải)
            const auto ss = g_app.safety_auditor.getState();
            using SS = std::decay_t<decltype(ss)>;
            const char* chip = "NOMINAL";
            ImU32 chip_col = kOliveBright;
            if (ss == SS::kPreAlert) { chip = "PRE-ALERT"; chip_col = kAmber; }
            else if (ss == SS::kDistressLocked) { chip = "SOS LOCK"; chip_col = kRed; }
            else if (ss == SS::kPanicCirclingWarning) { chip = "CIRCLING"; chip_col = kYellow; }
            char right[96];
            snprintf(right, sizeof(right), "FPS %3.0f   PWR %3d%%   LINK %s", io.Framerate,
                     static_cast<int>(g_app.battery_soc),
                     desktop::getWebServer()->isRunning() ? "WEB" : "LOCAL");
            const float right_w = ImGui::CalcTextSize(right).x;
            const float chip_w = ImGui::CalcTextSize(chip).x + 22.0F;
            ImGui::SameLine(bar_w - right_w - chip_w - 36.0F);
            {
                const ImVec2 p = ImGui::GetCursorScreenPos();
                const float h = ImGui::GetTextLineHeight();
                const bool alert = ss != SS::kNormal;
                const float a = alert ? 0.55F + 0.45F * std::sin(static_cast<float>(ImGui::GetTime()) * 5.0F) : 1.0F;
                ImDrawList* dl = ImGui::GetWindowDrawList();
                dl->AddRect(ImVec2(p.x - 4, p.y - 2), ImVec2(p.x + chip_w - 4, p.y + h + 2), chip_col);
                dl->AddRectFilled(ImVec2(p.x + 2, p.y + h * 0.5F - 3), ImVec2(p.x + 8, p.y + h * 0.5F + 3),
                                  withAlpha(chip_col, static_cast<int>(255 * a)));
                ImGui::SetCursorScreenPos(ImVec2(p.x + 14, p.y));
                ImGui::PushStyleColor(ImGuiCol_Text, v4(chip_col));
                ImGui::TextUnformatted(chip);
                ImGui::PopStyleColor();
            }
            ImGui::SameLine(bar_w - right_w - 14.0F);
            ImGui::PushStyleColor(ImGuiCol_Text, v4(kMuted));
            ImGui::TextUnformatted(right);
            ImGui::PopStyleColor();
            ImGui::EndMainMenuBar();
        }
        ImGui::PopStyleVar();

        // Các panel
        desktop::renderMapPanel(g_app, g_map_view);
        desktop::renderTelemetryPanel(g_app);
        desktop::renderSafetyPanel(g_app);
        desktop::renderRescuePanel(g_app);
        desktop::renderControlsPanel(g_app);
        desktop::renderReplayPanel(g_app);
        desktop::renderWebPanel(g_app);

        // Rendering
        ImGui::Render();
        const float clear[4] = {14.0F / 255.0F, 16.0F / 255.0F, 15.0F / 255.0F, 1.0F};
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        g_pSwapChain->Present(1, 0);  // vsync
    }

    desktop::getWebServer()->stop();  // [W3] tắt web server gọn khi thoát
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    CleanupDeviceD3D();
    DestroyWindow(g_hwnd);
    return 0;
}

int main() {
    // WinMain (không dùng main() để tránh required console subsystem nhập_lệch)
    STARTUPINFOW si = {};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_SHOWDEFAULT;
    return wWinMain(GetModuleHandleW(nullptr), nullptr, nullptr, SW_SHOWDEFAULT);
}
