cmake_minimum_required(VERSION 3.16)
project(dwa_local_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# The OpenCV visualization is optional: dwa_core deliberately has no graphics
# dependency, so the planner and its tests build and run headless without it.
option(DWA_WITH_GUI "Build the OpenCV visualization" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if(DWA_WITH_GUI)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
endif()

# Warnings are on by default: several of the bugs this project started with
# (uninitialised reads, unused members, sign mismatches) are things the
# compiler will point at.
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Algorithm library: no OpenCV, no I/O, so it can be unit-tested ---
add_library(dwa_core
  src/planner.cpp
  src/scenarios.cpp
)
target_include_directories(dwa_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(dwa_core PUBLIC Eigen3::Eigen)

# --- Visualization layer (optional) ---
if(DWA_WITH_GUI)
  add_library(dwa_viz src/visualizer.cpp)
  target_link_libraries(dwa_viz PUBLIC dwa_core ${OpenCV_LIBS})
  target_include_directories(dwa_viz PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

# --- Executable ---
add_executable(dwa_local_planner src/main.cpp)
if(DWA_WITH_GUI)
  target_link_libraries(dwa_local_planner PRIVATE dwa_core dwa_viz)
  target_compile_definitions(dwa_local_planner PRIVATE DWA_WITH_GUI)
else()
  target_link_libraries(dwa_local_planner PRIVATE dwa_core)
  message(STATUS "DWA_WITH_GUI=OFF: building headless, --no-gui is implied.")
endif()

# --- Tests ---
option(DWA_BUILD_TESTS "Build the unit tests" ON)

if(DWA_BUILD_TESTS)
  include(FetchContent)
  find_package(GTest QUIET)

  # Module mode sets GTEST_FOUND, config mode sets GTest_FOUND; checking the
  # imported target covers both.
  if(NOT TARGET GTest::gtest_main)
    message(STATUS "GTest not found locally; fetching it.")
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  enable_testing()

  add_executable(dwa_tests
    tests/test_angles.cpp
    tests/test_planner.cpp
  )
  target_link_libraries(dwa_tests PRIVATE dwa_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(dwa_tests)
endif()
