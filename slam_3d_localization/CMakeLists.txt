cmake_minimum_required(VERSION 3.8)
project(slam_3d_localization)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_3d_core REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)

add_library(slam_3d_localizer
  src/cloud_freshness_guard.cpp
  src/constant_velocity_predictor.cpp
  src/map_odom_transform_cache.cpp
  src/odom_pose_buffer.cpp
  src/slam_3d_localizer.cpp
)
target_include_directories(slam_3d_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(slam_3d_localizer
  diagnostic_msgs geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs
  std_srvs tf2 tf2_ros
)
target_link_libraries(slam_3d_localizer
  slam_3d_core::voxel_surfel_registration
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(slam_3d_localizer_node src/slam_3d_localizer_node.cpp)
ament_target_dependencies(slam_3d_localizer_node rclcpp)
target_link_libraries(slam_3d_localizer_node slam_3d_localizer)

install(TARGETS slam_3d_localizer slam_3d_localizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/estimate_imu_open_loop.py
  scripts/publish_localization_inputs.py
  scripts/resolve_latest_slam_stage.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_cloud_freshness_guard
    test/test_cloud_freshness_guard.cpp
  )
  target_link_libraries(test_cloud_freshness_guard slam_3d_localizer)
  ament_add_gtest(
    test_constant_velocity_predictor
    test/test_constant_velocity_predictor.cpp
  )
  target_link_libraries(test_constant_velocity_predictor slam_3d_localizer)
  ament_add_gtest(
    test_map_odom_transform_cache
    test/test_map_odom_transform_cache.cpp
  )
  target_link_libraries(test_map_odom_transform_cache slam_3d_localizer)
  ament_add_gtest(test_odom_pose_buffer test/test_odom_pose_buffer.cpp)
  target_link_libraries(test_odom_pose_buffer slam_3d_localizer)
  ament_add_pytest_test(test_localization_config test/test_localization_config.py)
  ament_add_pytest_test(
    test_imu_open_loop_estimator
    test/test_imu_open_loop_estimator.py
  )
  ament_add_pytest_test(
    test_publish_localization_inputs
    test/test_publish_localization_inputs.py
  )
  ament_add_pytest_test(
    test_resolve_latest_slam_stage
    test/test_resolve_latest_slam_stage.py
    TIMEOUT 90
  )
endif()

ament_package()
