#include <rclcpp/rclcpp.hpp>
#include "interfaces_demo/msg/key_value.hpp"
#include "interfaces_demo/msg/dictionary.hpp"


/*
1. QoS depth 0: The specified depth is 0, but due to the use of keep_all(), the subscriber actually receives all historical messages.
2. keep_all(): The subscriber will receive all published messages of the topic.
3. transient_local(): Even messages published before the subscriber is created will be received, provided that the lifecycles of the publisher and subscriber overlap.
4. reliable(): The message transmission is reliable, ensuring that all messages are successfully received, which is suitable for situations with high requirements for message integrity.
*/
class Sub : public rclcpp::Node
{
public:
    Sub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Node is running.");
        // 3. Create a subscriber
        
        sub = this->create_subscription<interfaces_demo::msg::Dictionary>("topic_msg", rclcpp::QoS(10).keep_all().transient_local().reliable(),
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
