cmake_minimum_required(VERSION 3.16)
project(mcl_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Algorithm library: no ROS dependency at all ---
# This is what makes the models unit-testable, and what would have made the
# ROS 2 port nearly free had it existed from the start.
add_library(mcl_core
  src/occupancy_grid.cpp
  src/likelihood_field.cpp
  src/motion_model.cpp
  src/sensor_model.cpp
  src/particle_filter.cpp
)
target_include_directories(mcl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- ROS 2 node ---
add_library(mcl_node_lib src/mcl_node.cpp)
target_link_libraries(mcl_node_lib PUBLIC mcl_core)
ament_target_dependencies(mcl_node_lib PUBLIC
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(mcl_localization_node src/main.cpp)
target_link_libraries(mcl_localization_node PRIVATE mcl_node_lib)

# --- Offline evaluation harness ---
# Links mcl_core only, so it builds and runs without a ROS graph. Ships in
# place of a recorded bag: reproducible from a seed, with known ground truth.
add_executable(mcl_evaluate tools/evaluate.cpp)
target_link_libraries(mcl_evaluate PRIVATE mcl_core)

install(TARGETS mcl_core mcl_node_lib
  EXPORT export_mcl_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mcl_localization_node mcl_evaluate
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config map tools DESTINATION share/${PROJECT_NAME})

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mcl_core
    test/test_angles.cpp
    test/test_occupancy_grid.cpp
    test/test_likelihood_field.cpp
    test/test_motion_model.cpp
    test/test_sensor_model.cpp
    test/test_particle_filter.cpp
  )
  if(TARGET test_mcl_core)
    target_link_libraries(test_mcl_core mcl_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_mcl_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
                          tf2_geometry_msgs)

ament_package()
