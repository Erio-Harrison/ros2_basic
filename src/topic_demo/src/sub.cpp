#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_demo/msg/person_info.hpp"
class Sub : public rclcpp::Node
{
public:
    Sub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 3. 创建订阅者
        sub = this->create_subscription<interfaces_demo::msg::PersonInfo>("name", 10,
                                                                          std::bind(&Sub::sub_callback, this, std::placeholders::_1));
    }

private:
    // 1.声明订阅者
    rclcpp::Subscription<interfaces_demo::msg::PersonInfo>::SharedPtr sub;
    // 2.订阅者回调函数
    void sub_callback(const interfaces_demo::msg::PersonInfo::SharedPtr msgs)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving name: %s, age: %d",
                    msgs->name.c_str(),
                    msgs->age);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sub>("Pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}