#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
rclcpp::init(argc, argv); //initialize  rclcpp
auto node = std::make_shared<rclcpp::Node>("node_01"); // Create node_01 node
RCLCPP_INFO(node->get_logger(), "node_01 is running"); // printout
rclcpp::spin(node); // Loop through nodes
rclcpp::shutdown(); // stop running
return 0;
}