cmake_minimum_required(VERSION 3.20)

add_library(survival_harness STATIC
    src/synthetic_generator.cpp
    src/data_player.cpp
    src/metrics_collector.cpp
    src/failure_injector.cpp
    src/benchmark_runner.cpp
    src/csv_loader.cpp
)

target_include_directories(survival_harness PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(survival_harness PUBLIC
    survival_core
    Eigen3::Eigen
)

target_compile_features(survival_harness PUBLIC cxx_std_20)
