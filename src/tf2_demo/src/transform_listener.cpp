#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TransformListenerNode : public rclcpp::Node
{
public:
  TransformListenerNode() : Node("transform_listener_node")
  {
    // 创建 tf2 buffer，这是必需的用于存储和管理接收到的变换
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // 创建 TransformListener，它会自动订阅 /tf 和 /tf_static 话题
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建一个定时器，定期查询变换
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TransformListenerNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // 查询动态变换
    geometry_msgs::msg::TransformStamped dynamic_transform;
    try {
      dynamic_transform = tf_buffer_->lookupTransform("world", "dynamic_frame", tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "Dynamic Transform: [%f, %f, %f]",
        dynamic_transform.transform.translation.x,
        dynamic_transform.transform.translation.y,
        dynamic_transform.transform.translation.z);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform from 'world' to 'dynamic_frame': %s", ex.what());
    }

    // 查询静态变换
    geometry_msgs::msg::TransformStamped static_transform;
    try {
      static_transform = tf_buffer_->lookupTransform("world", "static_frame", tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "Static Transform: [%f, %f, %f]",
        static_transform.transform.translation.x,
        static_transform.transform.translation.y,
        static_transform.transform.translation.z);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform from 'world' to 'static_frame': %s", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformListenerNode>());
  rclcpp::shutdown();
  return 0;
}