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
      std::bind(&BaseLinkBroadcaster::broadcastBaseLink, this));
  }

private:
  void broadcastBaseLink()
  {
    auto message = geometry_msgs::msg::TransformStamped();

    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";

    // Set the transformation values here.
    message.transform.translation.x = 0.0;
    message.transform.translation.y = 0.0;
    message.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

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
  rclcpp::spin(std::make_shared<BaseLinkBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
