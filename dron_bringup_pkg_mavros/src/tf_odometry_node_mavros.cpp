#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class TfOdometryNode : public rclcpp::Node
{
public:
  TfOdometryNode()
  : Node("tf_odometry_node_mavros")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    odom_msg_topic_ = declare_parameter<std::string>("odom_msg_topic", "/odom");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);
    publish_odom_msg_ = declare_parameter<bool>("publish_odom_msg", true);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    if (publish_map_to_odom_) {
      publish_static_map_to_odom();
    }

    if (publish_odom_msg_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_msg_topic_, 10);
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&TfOdometryNode::odom_cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "tf_odometry_node_mavros MAVROS iniciado. Subscrito a %s. Publicando TF %s -> %s",
      odom_topic_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str());
  }

private:
  void publish_static_map_to_odom()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(
      get_logger(),
      "TF estático publicado: %s -> %s",
      map_frame_.c_str(),
      odom_frame_.c_str());
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto stamp = now();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);

    if (publish_odom_msg_) {
      auto odom_msg = *msg;
      odom_msg.header.stamp = stamp;
      odom_msg.header.frame_id = odom_frame_;
      odom_msg.child_frame_id = base_frame_;
      odom_pub_->publish(odom_msg);
    }
  }

  std::string odom_topic_;
  std::string odom_msg_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_map_to_odom_{true};
  bool publish_odom_msg_{true};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
