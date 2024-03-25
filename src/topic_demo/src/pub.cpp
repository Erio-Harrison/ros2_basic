#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_demo/msg/person_info.hpp"
class Pub : public rclcpp::Node
{
public:
    Pub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 2.Create publisher
        pub = this->create_publisher<interfaces_demo::msg::PersonInfo>("name", 10);
        // Create timer to publish information
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&Pub::send_msg, this));
    }

private:
    // 1. Claim publisher
    rclcpp::Publisher<interfaces_demo::msg::PersonInfo>::SharedPtr pub;
    // 3.release news
    void send_msg()
    { 

        interfaces_demo::msg::PersonInfo info;
        // Create message
        info.symbol = "XBTUSD";
        info.id = 25585432705;
        info.side = "Sell";
        info.timestamp = "2024-03-24T13:50:14.196Z";
        info.size = 34000;
        info.price = 65479;
        // Log printing
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", info.symbol.c_str());

        // make an announcement
        pub->publish(info);
    }
    // 4. declare timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pub>("Pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}