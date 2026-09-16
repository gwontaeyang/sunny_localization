// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    const auto node = std::make_shared<gole::slam_3d_localization::LioOdometry>(
      rclcpp::NodeOptions().use_clock_thread(false));
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("slam_3d_lio_odometry_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
