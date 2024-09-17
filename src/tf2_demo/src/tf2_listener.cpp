#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TF2Listener : public rclcpp::Node
{
public:
  TF2Listener() : Node("tf2_listener")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TF2Listener::on_timer, this));
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      transformStamped = tf_buffer_->lookupTransform(
        "world", "robot",
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Robot position: x=%.2f, y=%.2f, z=%.2f",
      transformStamped.transform.translation.x,
      transformStamped.transform.translation.y,
      transformStamped.transform.translation.z);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF2Listener>());
  rclcpp::shutdown();
  return 0;
}