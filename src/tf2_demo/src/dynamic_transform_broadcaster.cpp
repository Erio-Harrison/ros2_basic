#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
  DynamicFrameBroadcaster() : Node("dynamic_frame_broadcaster")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DynamicFrameBroadcaster::broadcast_transform, this));
  }

private:
  void broadcast_transform()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "dynamic_frame";

    // 让frame随时间移动（这里是一个简单的圆周运动）
    double seconds = this->get_clock()->now().seconds();
    t.transform.translation.x = cos(seconds);
    t.transform.translation.y = sin(seconds);
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, seconds);  // 绕Z轴旋转
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}