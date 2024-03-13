#include "rclcpp/rclcpp.hpp"
#include "interfaces_demo/srv/add_two_int.hpp"
class Server : public rclcpp::Node
{
public:
    Server(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 3.创建服务端
        server = this->create_service<interfaces_demo::srv::AddTwoInt>("service", std::bind(&Server::server_callback, this,
                                                                                            std::placeholders::_1, std::placeholders::_2));
    }

private:
    // 1.声明服务端
    rclcpp::Service<interfaces_demo::srv::AddTwoInt>::SharedPtr server;
    // 2.服务端回调函数
    void server_callback(const interfaces_demo::srv::AddTwoInt::Request::SharedPtr request,
                         const interfaces_demo::srv::AddTwoInt::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving a request");
        response->sum = request->num1 + request->num2;
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Server>("server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}