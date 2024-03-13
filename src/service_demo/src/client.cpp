#include "rclcpp/rclcpp.hpp"
#include "interfaces_demo/srv/add_two_int.hpp"
class Client : public rclcpp::Node
{
public:
    Client(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running");
        // 3.创建客户端
        client = this->create_client<interfaces_demo::srv::AddTwoInt>("service");
        // 6.发送请求
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&Client::send_request, this));
    }

private:
    // 1.声明客户端
    rclcpp::Client<interfaces_demo::srv::AddTwoInt>::SharedPtr client;
    // 2.客户端回调函数
    void
    client_callback(rclcpp::Client<interfaces_demo::srv::AddTwoInt>::SharedFuture response)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving a response");
        auto result = response.get();
        RCLCPP_INFO(this->get_logger(), "sum: %d", result->sum);
    }
    // 4.发送请求函数
    void send_request()
    {
        while (!client->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for server···");
        }
        // 构造 request
        auto request =
            std::make_shared<interfaces_demo::srv::AddTwoInt_Request>();
        request->num1 = 10;
        request->num2 = 20;
        // 发送异步数据
        client->async_send_request(request,
                                   std::bind(&Client::client_callback, this, std::placeholders::_1));
    }
    // 5.声明定时器
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Client>("client");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}