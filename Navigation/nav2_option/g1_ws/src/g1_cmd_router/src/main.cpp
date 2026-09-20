#include <rclcpp/rclcpp.hpp>

#include "g1_cmd_router/cmd_router_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_cmd_router::CmdRouterNode>());
    rclcpp::shutdown();
    return 0;
}
