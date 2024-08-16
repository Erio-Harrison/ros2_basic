#include <rclcpp/rclcpp.hpp>
#include "interfaces_demo/msg/key_value.hpp"
#include "interfaces_demo/msg/dictionary.hpp"

class Sub : public rclcpp::Node
{
public:
    Sub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Node is running.");
        // 3. Create a subscriber
        sub = this->create_subscription<interfaces_demo::msg::Dictionary>("name", 10,
                std::bind(&Sub::sub_callback, this, std::placeholders::_1));
    }

private:
    // 1. Declare subscriber
    rclcpp::Subscription<interfaces_demo::msg::Dictionary>::SharedPtr sub;

    // 2. Subscriber callback function
    void sub_callback(const interfaces_demo::msg::Dictionary::SharedPtr msg)
    {
        for (const auto &kv : msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "Receiving key: %s, value: %s",
                        kv.key.c_str(), kv.value.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sub>("Sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
