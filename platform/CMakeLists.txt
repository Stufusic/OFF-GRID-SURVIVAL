cmake_minimum_required(VERSION 3.20)

add_library(survival_platform STATIC
    src/platform_memory_map.cpp
    src/platform_file_system.cpp
)

target_include_directories(survival_platform PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# survival_platform phụ thuộc survival_core (để lấy interface header)
# survival_core TUYỆT ĐỐI KHÔNG phụ thuộc survival_platform
target_link_libraries(survival_platform PUBLIC
    survival_core
)

target_compile_features(survival_platform PUBLIC cxx_std_20)
