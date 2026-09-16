cmake_minimum_required(VERSION 3.8)
project(odom_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Debug mode configuration
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG_MODE)
  message(STATUS "Building in Debug mode - Debug logging enabled")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gole_utility REQUIRED)
find_package(gole_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find all cpp files in src directory
file(GLOB_RECURSE CPP_SRC
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME}_node ${CPP_SRC})
target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  visualization_msgs
  ament_index_cpp
  gole_utility
  gole_interfaces
)

target_link_libraries(${PROJECT_NAME}_node
  gole_utility::gole_logger
  gole_utility::gole_diagnostic
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_imu_bias_config
    test/test_imu_bias_config.cpp
  )
  target_include_directories(test_imu_bias_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_imu_bias_config yaml-cpp)

  ament_add_gtest(test_imu_transform
    test/test_imu_transform.cpp
  )
  target_include_directories(test_imu_transform PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_imu_transform
    geometry_msgs
  )
endif()

# Install executables
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install models directory
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
