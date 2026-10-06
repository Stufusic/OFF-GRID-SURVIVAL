cmake_minimum_required(VERSION 3.20)

add_executable(unit_tests
    test_math_integrity.cpp
    test_data_contracts.cpp
    test_data_player.cpp
    test_metrics_collector.cpp
    test_eskf_3d.cpp
    test_stride_models.cpp
    test_step_detection.cpp
    test_pdr_integration.cpp
    test_dem_map.cpp
    test_dem_corruption.cpp
    test_local_blackbox.cpp
    test_jean_meeus.cpp
    test_particle_filter.cpp
    test_multi_hypothesis_clustering.cpp
    test_failure_injector.cpp
    test_multi_modal_fusion.cpp
    test_factorial_benchmark.cpp
    test_dem_mmap_io.cpp
    test_map_manager_gps_handoff.cpp
    test_safety_tumble_auditor.cpp
    test_resource_governor.cpp
    test_lockfree_c_api.cpp
    test_native_gui_math.cpp
    test_corner_cases_fuzzing.cpp
    test_csv_loader.cpp
    test_quaternion_conventions.cpp
    test_rescue_packet_e2e.cpp
    test_fault_injection_extended.cpp
    test_safety_invariants.cpp
    test_spatial_hash_benchmark.cpp
    test_realdata_regression.cpp
    test_desktop_hud.cpp
    test_gnss_stride_calibration.cpp
    test_web_json.cpp
    ../desktop/src/web_server.cpp  # [W4] buildStateJson/jsonEscape
    test_deterministic_replay.cpp
    test_ribbon_mesh.cpp
    test_stair_geometry_and_landing.cpp
    test_vehicle_spectral_and_motion_model.cpp
    test_escalator_and_vertical_transport.cpp
    test_motion_activity_fsm.cpp
    test_instantaneous_velocity_continuous.cpp
    test_phase1_hardening.cpp
    test_audit_v22_deep_dive.cpp
    test_scenario_harness.cpp
    test_probabilistic_stair_engine.cpp
)

target_include_directories(unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/desktop/src
    # httplib đã populate bởi desktop/CMakeLists (FetchContent dùng chung _deps)
    ${CMAKE_SOURCE_DIR}/build/_deps/httplib-src
)

target_link_libraries(unit_tests PRIVATE ws2_32
    
    survival_core
    survival_platform
    survival_harness
    GTest::gtest_main
)

target_compile_features(unit_tests PRIVATE cxx_std_20)

# [W4] httplib include cho test_web_json (httplib populate tại desktop/)
if(NOT httplib_SOURCE_DIR)
    FetchContent_Declare(httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG v0.15.3)
    FetchContent_GetProperties(httplib)
    if(NOT httplib_POPULATED)
        FetchContent_Populate(httplib)
    endif()
endif()

include(FetchContent)
include(GoogleTest)
gtest_discover_tests(unit_tests)
