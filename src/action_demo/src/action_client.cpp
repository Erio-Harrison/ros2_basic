#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces_demo/action/progress.hpp"
class ActionClient : public rclcpp::Node
{
public:
    ActionClient(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "ActionClient is running.");
        client =
            rclcpp_action::create_client<interfaces_demo::action::Progress>(this,
                                                                            "get_sum");
        send_goal(10);
    }

private:
    // 声明动作客户端
    rclcpp_action::Client<interfaces_demo::action::Progress>::SharedPtr
        client;
    void send_goal(int num)
    {
        // 连接服务端
        if (!client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "服务器连接失败!");
            return;
        }
        interfaces_demo::action::Progress::Goal goal;
        goal.num = num;
        rclcpp_action::Client<interfaces_demo::action::Progress>::SendGoalOptions
            options;
        options.goal_response_callback =
            std::bind(&ActionClient::goal_response_callback, this,
                      std::placeholders::_1);
        options.feedback_callback =
            std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1,
                      std::placeholders::_2);
        options.result_callback = std::bind(&ActionClient::result_callback,
                                            this, std::placeholders::_1);
        // 发送请求
        client->async_send_goal(goal, options);
    }
    // 目标响应
    void
    goal_response_callback(rclcpp_action::ClientGoalHandle<interfaces_demo::action::Progress>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "目标请求被拒绝!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标正在被处理中");
        }
    }
    // 反馈响应
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<interfaces_demo::action::Progress>::SharedPtr goal_handle,
        const std::shared_ptr<const interfaces_demo::action::Progress::Feedback> feedback)
    {
        double progress = feedback->progress;
        RCLCPP_INFO(this->get_logger(), "当前进度%.2f%%", progress);
    }
    // 结果响应
    void result_callback(const rclcpp_action::ClientGoalHandle<interfaces_demo::action::Progress>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "最终结果是:%d", result.result->sum);
        }
        else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(), "被中断");
        }
        else if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(), "被取消");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "未知错误");
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClient>("ActionClient");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}