#include <rclcpp/rclcpp.hpp>
#include "interfaces_demo/msg/key_value.hpp"
#include "interfaces_demo/msg/dictionary.hpp"

class Pub : public rclcpp::Node
{
public:
    Pub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Node is running.");
        // 2. Create publisher
        pub = this->create_publisher<interfaces_demo::msg::Dictionary>("topic_msg", 
                                                                        rclcpp::QoS(10).transient_local().reliable());

        // Create timer to publish information
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&Pub::send_msg, this));
    }

private:
    // 1. Declare publisher
    rclcpp::Publisher<interfaces_demo::msg::Dictionary>::SharedPtr pub;

    // 3. Publish messages
    void send_msg()
    { 
        interfaces_demo::msg::Dictionary dict_msg;

        // Create first KeyValue pair
        interfaces_demo::msg::KeyValue kv1;
        kv1.key = "symbol";
        kv1.value = "XBTUSD";
        dict_msg.data.push_back(kv1);

        kv1.key = "id";
        kv1.value = "25585432705";
        dict_msg.data.push_back(kv1);

        kv1.key = "side";
        kv1.value = "Sell";
        dict_msg.data.push_back(kv1);

        kv1.key = "timestamp";
        kv1.value = "2024-03-24T13:50:14.196Z";
        dict_msg.data.push_back(kv1);

        kv1.key = "size";
        kv1.value = "34000";
        dict_msg.data.push_back(kv1);

        kv1.key = "price";
        kv1.value = "65479";
        dict_msg.data.push_back(kv1);

        // Log the publishing action
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", kv1.value.c_str());

        // Create second KeyValue pair and add to the dictionary
        interfaces_demo::msg::KeyValue kv2;
        kv2.key = "symbol";
        kv2.value = "ASDFG";
        dict_msg.data.push_back(kv2);

        kv2.key = "id";
        kv2.value = "1584";
        dict_msg.data.push_back(kv2);

        kv2.key = "side";
        kv2.value = "Sold";
        dict_msg.data.push_back(kv2);

        kv2.key = "timestamp";
        kv2.value = "2024-03-24T88:50:14.196Z";
        dict_msg.data.push_back(kv2);

        kv2.key = "size";
        kv2.value = "987456";
        dict_msg.data.push_back(kv2);

        kv2.key = "price";
        kv2.value = "89522";
        dict_msg.data.push_back(kv2);

        // Log the publishing action
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", kv2.value.c_str());

        // Publish the dictionary message
        pub->publish(dict_msg);
    }

    // 4. Declare timer
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
