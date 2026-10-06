cmake_minimum_required(VERSION 3.20)

project(OffGridSurvivalEngine
    VERSION 2.2.0
    DESCRIPTION "Probabilistic Offline Survival and Navigation Engine"
    LANGUAGES CXX
)

# Standard C++20 enforcement
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Strict warning flags per Rule.md
if(MSVC)
    add_compile_options(/W4 /permissive- /Zc:__cplusplus /utf-8 /wd5054 /wd4127)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast)
endif()

# Dependencies using FetchContent
include(FetchContent)

# 1. Eigen 3.4.0
message(STATUS "Fetching Eigen 3.4.0...")
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE INTERNAL "")
set(BUILD_TESTING_SAVED ${BUILD_TESTING})
set(BUILD_TESTING OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(eigen)
set(BUILD_TESTING ${BUILD_TESTING_SAVED} CACHE BOOL "" FORCE)

# 2. GoogleTest 1.14.0
message(STATUS "Fetching GoogleTest 1.14.0...")
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    GIT_SHALLOW TRUE
)
# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Include subdirectories
add_subdirectory(core)
add_subdirectory(platform)
add_subdirectory(harness)
add_subdirectory(tests)
add_subdirectory(desktop)
add_subdirectory(tools)
