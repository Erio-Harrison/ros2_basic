#include "rclcpp/rclcpp.hpp"
#include "interfaces_demo/srv/add_two_int.hpp"
class Client : public rclcpp::Node
{
public:
    Client(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running");
        // 3.Creating a Client
        client = this->create_client<interfaces_demo::srv::AddTwoInt>("service");
        // 6.Send Request
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&Client::send_request, this));
    }

private:
    // 1.Declarative Client
    rclcpp::Client<interfaces_demo::srv::AddTwoInt>::SharedPtr client;
    // 2.Client callback function
    void
    client_callback(rclcpp::Client<interfaces_demo::srv::AddTwoInt>::SharedFuture response)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving a response");
        auto result = response.get();
        RCLCPP_INFO(this->get_logger(), "sum: %ld", result->sum);
    }
    // 4.Send request function
    void send_request()
    {
        while (!client->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for server···");
        }
        // Constructing request
        auto request =
            std::make_shared<interfaces_demo::srv::AddTwoInt_Request>();
        request->num1 = 10;
        request->num2 = 20;
        // Sending asynchronous data
        client->async_send_request(request,
                                   std::bind(&Client::client_callback, this, std::placeholders::_1));
    }
    // 5.Declare timer
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