cmake_minimum_required(VERSION 3.20)

add_library(survival_core STATIC
    src/integrity/integrity_guard.cpp
    src/integrity/integrity_monitor.cpp
    src/comms/packet_serializer.cpp
    src/comms/sos_egress_manager.cpp
    src/estimators/eskf_3d.cpp
    src/kinematics/adaptive_parametric_model.cpp
    src/kinematics/baseline_stride_models.cpp
    src/kinematics/step_detector.cpp
    src/kinematics/pdr_engine.cpp
    src/kinematics/stair_geometry_estimator.cpp
    src/kinematics/stair_motion_detector.cpp
    src/kinematics/vehicle_motion_detector.cpp
    src/kinematics/vehicle_motion_model.cpp
    src/kinematics/vertical_transport_detector.cpp
    src/kinematics/motion_transition_fsm.cpp
    src/kinematics/motion_activity_classifier.cpp
    src/kinematics/instantaneous_velocity_estimator.cpp
    src/environment/dem_map.cpp
    src/environment/jean_meeus_solver.cpp
    src/environment/map_manager.cpp
    src/estimators/particle_filter_engine.cpp
    src/estimators/multi_modal_fusion_pipeline.cpp
    src/safety/safety_auditor.cpp
    src/safety/resource_governor.cpp
    src/storage/local_blackbox.cpp
    src/c_api/survival_c_api.cpp
    ${CMAKE_SOURCE_DIR}/mobile/android/app/src/main/cpp/gui/egl/fps_governor.cpp
    ${CMAKE_SOURCE_DIR}/mobile/android/app/src/main/cpp/gui/renderer/ribbon_renderer.cpp
)

# [C1] File ESKF template-heavy (Eigen) vượt giới hạn 32768 sections COFF
# khi build Debug (-g) trên MinGW -> cần big-obj.
if(MINGW)
    set_source_files_properties(src/estimators/eskf_3d.cpp PROPERTIES
        COMPILE_OPTIONS "-Wa,-mbig-obj")
endif()

target_include_directories(survival_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/mobile/android/app/src/main/cpp/gui>
    $<INSTALL_INTERFACE:include>
)
# [M15] c_api (lớp cầu nối) gọi initializePlatformHooks — chỉ cần declaration header,
# symbol được resolve ở tầng cuối (tests/desktop đều link survival_platform)
target_include_directories(survival_core PRIVATE ${CMAKE_SOURCE_DIR}/platform/include)

target_link_libraries(survival_core PUBLIC
    Eigen3::Eigen
)

target_compile_features(survival_core PUBLIC cxx_std_20)
