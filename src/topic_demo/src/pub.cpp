#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_demo/msg/person_info.hpp"
class Pub : public rclcpp::Node
{
public:
    Pub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 2.创建发布者
        pub = this->create_publisher<interfaces_demo::msg::PersonInfo>("name", 10);
        // 创建定时器发布信息
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&Pub::send_msg, this));
    }

private:
    // 1. 声明发布者
    rclcpp::Publisher<interfaces_demo::msg::PersonInfo>::SharedPtr pub;
    // 3.发布信息
    void send_msg()
    { 
        interfaces_demo::msg::PersonInfo info2;
        info2.name = "li si";
        info2.age = 22;

        interfaces_demo::msg::PersonInfo info;
        // 创建消息
        info.name = "zhang san";
        info.age = 18;
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", info2.name.c_str());
        // 发布消息
        pub->publish(info2);
    }
    // 4. 声明定时器
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