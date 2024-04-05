#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class Zed2CameraLinkBroadcaster : public rclcpp::Node
{
public:
  Zed2CameraLinkBroadcaster() : Node("zed2_camera_link_broadcaster")
  {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Zed2CameraLinkBroadcaster::broadcastZed2CameraLink, this));
  }

private:
  void broadcastZed2CameraLink()
  {
    auto message = geometry_msgs::msg::TransformStamped();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    message.child_frame_id = "zed2_camera_link";
    message.transform.translation.x = 1.0; // Adjust these values as necessary
    message.transform.translation.y = 0.0;
    message.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // Adjust the rotation as necessary
    message.transform.rotation.x = q.x();
    message.transform.rotation.y = q.y();
    message.transform.rotation.z = q.z();
    message.transform.rotation.w = q.w();

    broadcaster_->sendTransform(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Zed2CameraLinkBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
