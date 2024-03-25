#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_demo/msg/person_info.hpp"
class Sub : public rclcpp::Node
{
public:
    Sub(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "node is running.");
        // 3. Create a subscriber
        sub = this->create_subscription<interfaces_demo::msg::PersonInfo>("name", 10,
                                                                          std::bind(&Sub::sub_callback, this, std::placeholders::_1));
    }

private:
    // 1.Declare subscribers
    rclcpp::Subscription<interfaces_demo::msg::PersonInfo>::SharedPtr sub;
    // 2.Subscriber callback function
    void sub_callback(const interfaces_demo::msg::PersonInfo::SharedPtr msgs)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving name: %s, id: %ld, side: %s \n                                     timestamp: %s, size: %ld, price: %ld",
                    msgs->symbol.c_str(),
                    msgs->id,
                    msgs->side.c_str(),
                    msgs->timestamp.c_str(),
                    msgs->size,
                    msgs->price);
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