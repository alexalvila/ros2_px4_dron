#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "dron_bringup_pkg/px4_qos.hpp"

class TfOdometryNode : public rclcpp::Node
{
public:
  TfOdometryNode()
  : Node("tf_odometry_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fmu/out/vehicle_odometry");
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

    odom_sub_ =
      create_subscription<px4_msgs::msg::VehicleOdometry>(
        odom_topic_,
        dron_bringup_pkg::px4_qos(),
        std::bind(&TfOdometryNode::odom_cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "tf_odometry_node iniciado. Subscrito a %s. Publicando TF %s -> %s",
      odom_topic_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str());
  }

private:
  static bool is_finite(float value)
  {
    return std::isfinite(static_cast<double>(value));
  }

  static bool valid_position(const std::array<float, 3> & p)
  {
    return is_finite(p[0]) && is_finite(p[1]) && is_finite(p[2]);
  }

  static bool valid_quaternion(const std::array<float, 4> & q)
  {
    return is_finite(q[0]) && is_finite(q[1]) && is_finite(q[2]) && is_finite(q[3]);
  }

  void publish_static_map_to_odom()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(
      get_logger(),
      "TF estático publicado: %s -> %s",
      map_frame_.c_str(),
      odom_frame_.c_str());
  }

  tf2::Quaternion px4_quat_ned_frd_to_ros_enu_flu(const std::array<float, 4> & q_px4_array)
  {
    /*
      PX4:
        mundo: NED  = North, East, Down
        cuerpo: FRD = Forward, Right, Down

      ROS:
        mundo: ENU  = East, North, Up
        cuerpo: FLU = Forward, Left, Up

      Conversión:
        R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu
    */

    const tf2::Quaternion q_ned_frd(
      q_px4_array[1],
      q_px4_array[2],
      q_px4_array[3],
      q_px4_array[0]);

    const tf2::Matrix3x3 r_ned_frd(q_ned_frd);

    const tf2::Matrix3x3 r_enu_ned(
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0);

    const tf2::Matrix3x3 r_frd_flu(
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0);

    const tf2::Matrix3x3 r_enu_flu = r_enu_ned * r_ned_frd * r_frd_flu;

    tf2::Quaternion q_enu_flu;
    r_enu_flu.getRotation(q_enu_flu);
    q_enu_flu.normalize();

    return q_enu_flu;
  }

  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (!valid_position(msg->position) || !valid_quaternion(msg->q)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "VehicleOdometry contiene NaN/Inf. Ignorando mensaje.");
      return;
    }

    const auto stamp = now();

    /*
      PX4 position:
        x = North
        y = East
        z = Down

      ROS ENU:
        x = East
        y = North
        z = Up
    */
    const double x_enu = static_cast<double>(msg->position[1]);
    const double y_enu = static_cast<double>(msg->position[0]);
    const double z_enu = -static_cast<double>(msg->position[2]);

    const tf2::Quaternion q_ros = px4_quat_ned_frd_to_ros_enu_flu(msg->q);

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = x_enu;
    tf_msg.transform.translation.y = y_enu;
    tf_msg.transform.translation.z = z_enu;

    tf_msg.transform.rotation.x = q_ros.x();
    tf_msg.transform.rotation.y = q_ros.y();
    tf_msg.transform.rotation.z = q_ros.z();
    tf_msg.transform.rotation.w = q_ros.w();

    tf_broadcaster_->sendTransform(tf_msg);

    if (publish_odom_msg_) {
      nav_msgs::msg::Odometry odom_msg;

      odom_msg.header.stamp = stamp;
      odom_msg.header.frame_id = odom_frame_;
      odom_msg.child_frame_id = base_frame_;

      odom_msg.pose.pose.position.x = x_enu;
      odom_msg.pose.pose.position.y = y_enu;
      odom_msg.pose.pose.position.z = z_enu;

      odom_msg.pose.pose.orientation.x = q_ros.x();
      odom_msg.pose.pose.orientation.y = q_ros.y();
      odom_msg.pose.pose.orientation.z = q_ros.z();
      odom_msg.pose.pose.orientation.w = q_ros.w();

      /*
        PX4 velocity suele venir en NED.
        ROS ENU:
          vx = East
          vy = North
          vz = Up
      */
      odom_msg.twist.twist.linear.x = static_cast<double>(msg->velocity[1]);
      odom_msg.twist.twist.linear.y = static_cast<double>(msg->velocity[0]);
      odom_msg.twist.twist.linear.z = -static_cast<double>(msg->velocity[2]);

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
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfOdometryNode>());
  rclcpp::shutdown();
  return 0;
}