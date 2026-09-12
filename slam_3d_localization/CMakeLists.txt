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

set(SLAM_3D_LOCALIZATION_FAST_GICP "AUTO" CACHE STRING
  "Optional fast_gicp support: AUTO, ON, or OFF")
set_property(CACHE SLAM_3D_LOCALIZATION_FAST_GICP PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${SLAM_3D_LOCALIZATION_FAST_GICP}" _fast_gicp_mode)
if(NOT _fast_gicp_mode MATCHES "^(AUTO|ON|OFF)$")
  message(FATAL_ERROR
    "SLAM_3D_LOCALIZATION_FAST_GICP must be AUTO, ON, or OFF")
endif()

set(_slam_3d_localization_has_fast_gicp FALSE)
if(NOT _fast_gicp_mode STREQUAL "OFF")
  find_package(fast_gicp QUIET CONFIG)
  if(fast_gicp_FOUND)
    set(_slam_3d_localization_has_fast_gicp TRUE)
    message(STATUS "slam_3d_localization: FastGICP CPU backend enabled")
  elseif(_fast_gicp_mode STREQUAL "ON")
    message(FATAL_ERROR
      "SLAM_3D_LOCALIZATION_FAST_GICP=ON, but fast_gicp was not found")
  else()
    message(STATUS
      "slam_3d_localization: fast_gicp not found; gicp and voxel_surfel remain available")
  endif()
endif()

option(SLAM_3D_LOCALIZATION_ENABLE_FAST_VGICP_CUDA
  "Compile the opt-in FastVGICPCuda backend after a link capability probe" OFF)
set(SLAM_3D_LOCALIZATION_FAST_GICP_REVISION "unknown" CACHE STRING
  "Pinned fast_gicp source revision recorded in startup diagnostics")
if(DEFINED CMAKE_CUDA_ARCHITECTURES AND NOT "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
  set(_default_fast_gicp_cuda_architectures "${CMAKE_CUDA_ARCHITECTURES}")
else()
  set(_default_fast_gicp_cuda_architectures "unknown-prebuilt")
endif()
set(SLAM_3D_LOCALIZATION_FAST_GICP_CUDA_ARCHITECTURES
  "${_default_fast_gicp_cuda_architectures}" CACHE STRING
  "CUDA architectures embedded in the separately built fast_gicp dependency")
set(_slam_3d_localization_cuda_architectures_definition
  "${SLAM_3D_LOCALIZATION_FAST_GICP_CUDA_ARCHITECTURES}")
string(REPLACE ";" "," _slam_3d_localization_cuda_architectures_definition
  "${_slam_3d_localization_cuda_architectures_definition}")

set(_slam_3d_localization_has_fast_vgicp_cuda FALSE)
if(SLAM_3D_LOCALIZATION_ENABLE_FAST_VGICP_CUDA)
  if(NOT _slam_3d_localization_has_fast_gicp)
    message(FATAL_ERROR
      "FastVGICPCuda was requested, but fast_gicp is unavailable")
  endif()
  include(CheckCXXSourceCompiles)
  set(_saved_required_includes "${CMAKE_REQUIRED_INCLUDES}")
  set(_saved_required_libraries "${CMAKE_REQUIRED_LIBRARIES}")
  set(CMAKE_REQUIRED_INCLUDES
    ${fast_gicp_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  set(CMAKE_REQUIRED_LIBRARIES ${fast_gicp_LIBRARIES} ${PCL_LIBRARIES})
  # CheckCXXSourceCompiles caches its result. Force a fresh probe so changing
  # the sourced fast_gicp overlay cannot reuse a result from another install.
  unset(SLAM_3D_LOCALIZATION_FAST_VGICP_CUDA_LINKS CACHE)
  unset(SLAM_3D_LOCALIZATION_FAST_VGICP_CUDA_LINKS)
  check_cxx_source_compiles(
    "#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
     #include <pcl/point_types.h>
     int main() {
       fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> registration;
       return registration.hasConverged() ? 0 : 0;
     }"
    SLAM_3D_LOCALIZATION_FAST_VGICP_CUDA_LINKS)
  set(CMAKE_REQUIRED_INCLUDES "${_saved_required_includes}")
  set(CMAKE_REQUIRED_LIBRARIES "${_saved_required_libraries}")
  if(NOT SLAM_3D_LOCALIZATION_FAST_VGICP_CUDA_LINKS)
    message(FATAL_ERROR
      "FastVGICPCuda was requested, but the installed fast_gicp library does not "
      "link the CUDA backend. Rebuild fast_gicp with BUILD_VGICP_CUDA=ON.")
  endif()
  find_package(CUDAToolkit REQUIRED)
  set(_slam_3d_localization_has_fast_vgicp_cuda TRUE)
  message(STATUS
    "slam_3d_localization: FastVGICPCuda backend link probe passed; dependency architectures=${SLAM_3D_LOCALIZATION_FAST_GICP_CUDA_ARCHITECTURES}")
endif()

add_library(slam_3d_localizer
  src/cloud_freshness_guard.cpp
  src/constant_velocity_predictor.cpp
  src/map_odom_transform_cache.cpp
  src/odom_pose_buffer.cpp
  src/registration_backend.cpp
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
if(_slam_3d_localization_has_fast_gicp)
  target_include_directories(slam_3d_localizer PRIVATE ${fast_gicp_INCLUDE_DIRS})
  target_link_libraries(slam_3d_localizer ${fast_gicp_LIBRARIES})
  target_compile_definitions(slam_3d_localizer PRIVATE
    SLAM_3D_LOCALIZATION_HAS_FAST_GICP=1
    "SLAM_3D_LOCALIZATION_FAST_GICP_REVISION=\"${SLAM_3D_LOCALIZATION_FAST_GICP_REVISION}\"")
endif()
if(_slam_3d_localization_has_fast_vgicp_cuda)
  target_link_libraries(slam_3d_localizer CUDA::cudart)
  target_compile_definitions(slam_3d_localizer PRIVATE
    SLAM_3D_LOCALIZATION_HAS_FAST_VGICP_CUDA=1
    "SLAM_3D_LOCALIZATION_CUDA_ARCHITECTURES=\"${_slam_3d_localization_cuda_architectures_definition}\"")
endif()

add_library(lio_state_estimator
  src/imu_initializer.cpp
  src/lio_state_estimator.cpp
)
target_include_directories(lio_state_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lio_state_estimator Eigen3::Eigen)

add_library(slam_3d_lio_odometry
  src/lio_odometry.cpp
)
target_include_directories(slam_3d_lio_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(slam_3d_lio_odometry
  diagnostic_msgs geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs
  tf2 tf2_ros
)
target_link_libraries(slam_3d_lio_odometry
  lio_state_estimator
  slam_3d_core::voxel_surfel_registration
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(slam_3d_localizer_node src/slam_3d_localizer_node.cpp)
ament_target_dependencies(slam_3d_localizer_node rclcpp)
target_link_libraries(slam_3d_localizer_node slam_3d_localizer)

add_executable(slam_3d_lio_odometry_node src/lio_odometry_node.cpp)
ament_target_dependencies(slam_3d_lio_odometry_node rclcpp)
target_link_libraries(slam_3d_lio_odometry_node slam_3d_lio_odometry)

install(TARGETS
  lio_state_estimator
  slam_3d_lio_odometry
  slam_3d_lio_odometry_node
  slam_3d_localizer
  slam_3d_localizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY patches DESTINATION share/${PROJECT_NAME})
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
  ament_add_gtest(test_registration_backend test/test_registration_backend.cpp)
  target_link_libraries(test_registration_backend slam_3d_localizer)
  ament_add_gtest(test_imu_initializer test/test_imu_initializer.cpp)
  target_link_libraries(test_imu_initializer lio_state_estimator)
  ament_add_gtest(test_lio_state_estimator test/test_lio_state_estimator.cpp)
  target_link_libraries(test_lio_state_estimator lio_state_estimator)
  ament_add_gtest(test_lio_point_time test/test_lio_point_time.cpp)
  target_link_libraries(test_lio_point_time slam_3d_lio_odometry)
  ament_add_gtest(test_lidar_only_odometry test/test_lidar_only_odometry.cpp TIMEOUT 60)
  target_link_libraries(test_lidar_only_odometry slam_3d_lio_odometry)
  set_tests_properties(test_lidar_only_odometry PROPERTIES
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=223")
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
