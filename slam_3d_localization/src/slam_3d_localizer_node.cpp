#include <exception>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "slam_3d_localization/slam_3d_localizer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(
      std::make_shared<gole::slam_3d_localization::Slam3dLocalizer>(
        rclcpp::NodeOptions()));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("slam_3d_localizer_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
