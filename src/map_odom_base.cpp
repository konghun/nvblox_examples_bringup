#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 


class OdomToBaseLinkBroadcaster : public rclcpp::Node
{
public:
    OdomToBaseLinkBroadcaster() : Node("odom_to_base_link_broadcaster")
    {
        // 오도메트리 데이터 구독
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OdomToBaseLinkBroadcaster::odom_callback, this, std::placeholders::_1));
        
        // TF 발행자 초기화
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 변환 메시지 생성
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        // 위치 정보
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        std::cout << msg->pose.pose.position.x << std::endl;
        std::cout << msg->pose.pose.position.y << std::endl;



        // 방향 정보 (쿼터니언)
        transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;


        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);  // 쿼터니언을 3x3 행렬로 변환
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);  // 오일러 각으로 변환

        // Yaw 값을 출력
        std::cout << "Yaw: " << yaw << std::endl;
        // TF 발행
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToBaseLinkBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
