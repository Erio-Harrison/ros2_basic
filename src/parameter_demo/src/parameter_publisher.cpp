#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class ParameterPublisher : public rclcpp::Node
{
public:
    ParameterPublisher()
        : Node("parameter_publisher"), count_(0), external_update_(false)
    {
        this->declare_parameter("my_parameter", "Hello World");

        // Set up callback for parameter changes
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterPublisher::parametersCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1000ms, std::bind(&ParameterPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto param = this->get_parameter("my_parameter");
        RCLCPP_INFO(this->get_logger(), "Current parameter value: '%s'", param.as_string().c_str());

        // Only update the parameter if there hasn't been an external update
        if (!external_update_ && count_ % 5 == 0) {
            std::string new_value = "Hello World " + std::to_string(count_);
            this->set_parameter(rclcpp::Parameter("my_parameter", new_value));
            RCLCPP_INFO(this->get_logger(), "Parameter updated internally to: '%s'", new_value.c_str());
        }

        count_++;
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "my_parameter")
            {
                RCLCPP_INFO(this->get_logger(), "Parameter updated externally to: '%s'", param.as_string().c_str());
                external_update_ = true;
                // You can add any additional logic here to handle the parameter change
            }
        }
        return result;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    size_t count_;
    bool external_update_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterPublisher>());
    rclcpp::shutdown();
    return 0;
}