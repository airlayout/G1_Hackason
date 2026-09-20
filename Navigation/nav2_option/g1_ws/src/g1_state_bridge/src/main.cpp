#include <rclcpp/rclcpp.hpp>

#include "g1_state_bridge/state_bridge_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_state_bridge::StateBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
