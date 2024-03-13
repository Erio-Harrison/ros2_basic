#include "rclcpp/rclcpp.hpp"
using namespace std;
class Node02 : public rclcpp::Node
{
    public:
        Node02(string name) : Node(name){
            RCLCPP_INFO(this->get_logger(), "node_01 is running");
        }
};

int main(int argc,char ** argv){

    rclcpp::init(argc,argv);
    auto node = make_shared<Node02>("Node02");
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}