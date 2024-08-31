#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

class ParameterSubscriber : public rclcpp::Node
{
public:
    ParameterSubscriber()
        : Node("parameter_subscriber")
    {
        subscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events",
            10,
            std::bind(&ParameterSubscriber::parameter_event_callback, this, std::placeholders::_1));
    }

private:
    void parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        for (auto & changed_parameter : event->changed_parameters) {
            if (changed_parameter.name == "my_parameter") {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received updated parameter: '%s'",
                    changed_parameter.value.string_value.c_str());
            }
        }
    }

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterSubscriber>());
    rclcpp::shutdown();
    return 0;
}