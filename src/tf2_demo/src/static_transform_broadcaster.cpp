#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class StaticFrameBroadcaster : public rclcpp::Node
{
public:
  StaticFrameBroadcaster() : Node("static_frame_broadcaster")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 发布静态变换
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "static_frame";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 3.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 1.57);  // 绕Z轴旋转90度
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}