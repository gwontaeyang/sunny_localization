#include <rclcpp/rclcpp.hpp>

#include "amcl_guard/amcl_guard.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amcl_guard::AmclGuard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
