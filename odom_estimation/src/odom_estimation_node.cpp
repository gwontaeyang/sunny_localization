#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "odom_estimation/odom_estimation.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<gole::odom_estimation::OdomEstimation>();
        
        RCLCPP_INFO(node->get_logger(), "Odom Estimation Node started");
        rclcpp::spin(node->get_node_base_interface());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("odom_estimation"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
