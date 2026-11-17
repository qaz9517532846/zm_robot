// Broadcasts the map -> base_link TF directly from /odom, since no SLAM/localization
// node is running yet. This treats the odom pose as if it were already in the map
// frame (zero offset) so that consumers requiring a "map" frame (RViz fixed frame,
// Nav2 global_frame, COMPASS goal poses) have one to resolve against.

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class MapTfBroadcaster : public rclcpp::Node
{
public:
  MapTfBroadcaster()
  : Node("map_tf_broadcaster")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&MapTfBroadcaster::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Broadcasting '%s' -> '%s' TF from odom topic '%s'",
      map_frame_.c_str(), base_frame_.c_str(), odom_topic.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom_msg->header.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom_msg->pose.pose.position.x;
    transform.transform.translation.y = odom_msg->pose.pose.position.y;
    transform.transform.translation.z = odom_msg->pose.pose.position.z;
    transform.transform.rotation = odom_msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  std::string map_frame_;
  std::string base_frame_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
