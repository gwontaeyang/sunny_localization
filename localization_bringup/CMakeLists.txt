cmake_minimum_required(VERSION 3.8)
project(localization_bringup)

find_package(ament_cmake REQUIRED)
find_package(gole_utility REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(localization_bringup_node
  src/localization_bringup_node.cpp
)

ament_target_dependencies(localization_bringup_node
  gole_utility
  lifecycle_msgs
  rclcpp
  std_msgs
)

target_link_libraries(localization_bringup_node
  gole_utility::gole_logger
  yaml-cpp
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

install(TARGETS
  localization_bringup_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
