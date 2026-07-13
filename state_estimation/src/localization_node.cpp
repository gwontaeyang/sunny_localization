#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "localization/localization.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<gole::localization::Localization>();
        
        RCLCPP_INFO(node->get_logger(), "Localization Node started");
        rclcpp::spin(node->get_node_base_interface());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimation"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
