cmake_minimum_required(VERSION 3.20)

include(FetchContent)

# ============================================================================
# [U0] Dear ImGui v1.90.9 + ImPlot v0.16 (FetchContent, backend Win32 + DX11)
# ============================================================================
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.9-docking
)
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
)
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.15.3
)
FetchContent_GetProperties(httplib)
if(NOT httplib_POPULATED)
    FetchContent_Populate(httplib)
endif()
add_library(httplib INTERFACE)
target_include_directories(httplib INTERFACE ${httplib_SOURCE_DIR})

FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()
FetchContent_GetProperties(implot)
if(NOT implot_POPULATED)
    FetchContent_Populate(implot)
endif()

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)

add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot PUBLIC imgui)

# ============================================================================
# survival_desktop_gui — Navigation Dashboard (thay thế monolith GDI+)
# ============================================================================
add_executable(survival_desktop_gui
    src/app_main.cpp
    src/ui_map.cpp
    src/ui_dashboard.cpp
    src/ui_replay.cpp
    src/ui_web.cpp
    src/web_server.cpp
)

set_property(TARGET survival_desktop_gui PROPERTY WIN32_EXECUTABLE TRUE)

target_include_directories(survival_desktop_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/core/include
    ${CMAKE_SOURCE_DIR}/harness/include
)

target_link_libraries(survival_desktop_gui PRIVATE
    imgui
    implot
    httplib
    ws2_32
    survival_core
    survival_platform
    survival_harness
    d3d11
    d3dcompiler
    dxgi
    dwmapi
)

target_compile_features(survival_desktop_gui PRIVATE cxx_std_20)

# ImGui/ImPlot không thuộc mục tiêu zero-warning của dự án
if(MSVC)
    target_compile_options(imgui PRIVATE /W0)
    target_compile_options(implot PRIVATE /W0)
else()
    target_compile_options(imgui PRIVATE -w)
    target_compile_options(implot PRIVATE -w)
endif()
