// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    const auto node = std::make_shared<gole::slam_3d_localization::LioOdometry>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("slam_3d_lio_odometry_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
