#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class BaseLinkBroadcaster : public rclcpp::Node
{
public:
  BaseLinkBroadcaster() : Node("base_link_broadcaster")
  {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BaseLinkBroadcaster::broadcastTransforms, this));
  }

private:
  void broadcastTransforms()
  {
    // Broadcast base_link relative to odom
    auto base_link_message = geometry_msgs::msg::TransformStamped();
    base_link_message.header.stamp = this->get_clock()->now();
    base_link_message.header.frame_id = "odom";
    base_link_message.child_frame_id = "base_link";
    base_link_message.transform.translation.x = 0.0;  // Adjust as necessary
    base_link_message.transform.translation.y = 0.0;
    base_link_message.transform.translation.z = 0.0;
    tf2::Quaternion base_link_q;
    base_link_q.setRPY(0, 0, 0);
    base_link_message.transform.rotation.x = base_link_q.x();
    base_link_message.transform.rotation.y = base_link_q.y();
    base_link_message.transform.rotation.z = base_link_q.z();
    base_link_message.transform.rotation.w = base_link_q.w();
    broadcaster_->sendTransform(base_link_message);

    // Broadcast zed2_camera_link relative to base_link
    auto zed2_camera_link_message = geometry_msgs::msg::TransformStamped();
    zed2_camera_link_message.header.stamp = this->get_clock()->now();
    zed2_camera_link_message.header.frame_id = "base_link";
    zed2_camera_link_message.child_frame_id = "zed2_camera_link";
    zed2_camera_link_message.transform.translation.x = 0.0;  // Adjust these values as necessary
    zed2_camera_link_message.transform.translation.y = 0.0;
    zed2_camera_link_message.transform.translation.z = 0.0;
    tf2::Quaternion zed2_q;
    zed2_q.setRPY(0, 0, 0);  // Adjust the rotation as necessary
    zed2_camera_link_message.transform.rotation.x = zed2_q.x();
    zed2_camera_link_message.transform.rotation.y = zed2_q.y();
    zed2_camera_link_message.transform.rotation.z = zed2_q.z();
    zed2_camera_link_message.transform.rotation.w = zed2_q.w();
    broadcaster_->sendTransform(zed2_camera_link_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseLinkBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
