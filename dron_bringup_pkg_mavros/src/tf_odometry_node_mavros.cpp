#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class TfOdometryNode : public rclcpp::Node
{
public:
  TfOdometryNode()
  : Node("tf_odometry_node_mavros")
  {
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic",
      "/mavros/local_position/odom"
    );

    odom_msg_topic_ = declare_parameter<std::string>(
      "odom_msg_topic",
      "/odom"
    );

    map_frame_ = declare_parameter<std::string>(
      "map_frame",
      "map"
    );

    odom_frame_ = declare_parameter<std::string>(
      "odom_frame",
      "odom"
    );

    base_frame_ = declare_parameter<std::string>(
      "base_frame",
      "base_link"
    );

    publish_map_to_odom_ = declare_parameter<bool>(
      "publish_map_to_odom",
      true
    );

    publish_odom_msg_ = declare_parameter<bool>(
      "publish_odom_msg",
      true
    );

    publish_identity_until_odom_ = declare_parameter<bool>(
      "publish_identity_until_odom",
      true
    );

    use_msg_timestamp_ = declare_parameter<bool>(
      "use_msg_timestamp",
      false
    );

    zero_initial_pose_ = declare_parameter<bool>(
      "zero_initial_pose",
      true
    );

    base_z_offset_ = declare_parameter<double>(
      "base_z_offset",
      0.0
    );

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    if (publish_map_to_odom_) {
      publish_static_map_to_odom();
    }

    if (publish_odom_msg_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        odom_msg_topic_,
        rclcpp::SensorDataQoS()
      );
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TfOdometryNode::odom_cb, this, std::placeholders::_1)
    );

    fallback_timer_ = create_wall_timer(
      50ms,
      std::bind(&TfOdometryNode::fallback_timer_cb, this)
    );

    RCLCPP_INFO(get_logger(), "tf_odometry_node_mavros iniciado");

    RCLCPP_INFO(
      get_logger(),
      "Subscrito a odometria MAVROS: %s",
      odom_topic_.c_str()
    );

    if (publish_map_to_odom_) {
      RCLCPP_INFO(
        get_logger(),
        "Publicando TF estatico: %s -> %s",
        map_frame_.c_str(),
        odom_frame_.c_str()
      );
    }

    RCLCPP_INFO(
      get_logger(),
      "Publicando TF dinamico: %s -> %s",
      odom_frame_.c_str(),
      base_frame_.c_str()
    );

    RCLCPP_INFO(
      get_logger(),
      "Publicando odometria para RViz en: %s",
      odom_msg_topic_.c_str()
    );

    RCLCPP_INFO(
      get_logger(),
      "zero_initial_pose: %s",
      zero_initial_pose_ ? "true" : "false"
    );

    RCLCPP_INFO(
      get_logger(),
      "base_z_offset: %.3f",
      base_z_offset_
    );
  }

private:
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
  }

  rclcpp::Time valid_stamp(const builtin_interfaces::msg::Time & stamp_msg)
  {
    if (!use_msg_timestamp_) {
      return now();
    }

    const rclcpp::Time stamp(stamp_msg);
    if (stamp.nanoseconds() == 0) {
      return now();
    }

    return stamp;
  }

  tf2::Transform odom_to_tf2(const nav_msgs::msg::Odometry & odom_msg)
  {
    tf2::Transform tf;

    tf2::Vector3 position(
      odom_msg.pose.pose.position.x,
      odom_msg.pose.pose.position.y,
      odom_msg.pose.pose.position.z
    );

    tf2::Quaternion orientation;
    tf2::fromMsg(odom_msg.pose.pose.orientation, orientation);

    if (orientation.length2() < 1e-9) {
      orientation.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      orientation.normalize();
    }

    tf.setOrigin(position);
    tf.setRotation(orientation);

    return tf;
  }

  nav_msgs::msg::Odometry make_relative_odom(
    const nav_msgs::msg::Odometry & odom_msg,
    const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom_out = odom_msg;

    odom_out.header.stamp = stamp;
    odom_out.header.frame_id = odom_frame_;
    odom_out.child_frame_id = base_frame_;

    tf2::Transform current_tf = odom_to_tf2(odom_msg);

    if (zero_initial_pose_) {
      if (!origin_initialized_) {
        initial_tf_ = current_tf;
        origin_initialized_ = true;

        const auto p = initial_tf_.getOrigin();
        const auto q = initial_tf_.getRotation();

        RCLCPP_INFO(
          get_logger(),
          "Origen local fijado con primera pose: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
          p.x(), p.y(), p.z(),
          q.x(), q.y(), q.z(), q.w()
        );
      }

      current_tf = initial_tf_.inverse() * current_tf;
    }

    tf2::Vector3 p = current_tf.getOrigin();

    odom_out.pose.pose.position.x = p.x();
    odom_out.pose.pose.position.y = p.y();
    odom_out.pose.pose.position.z = p.z() + base_z_offset_;

    odom_out.pose.pose.orientation = tf2::toMsg(current_tf.getRotation());

    return odom_out;
  }

  void publish_dynamic_tf(
    const rclcpp::Time & stamp,
    const nav_msgs::msg::Odometry & odom_msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publish_identity_tf()
  {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = base_z_offset_;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    publish_dynamic_tf(odom_msg.header.stamp, odom_msg);

    if (publish_odom_msg_) {
      odom_pub_->publish(odom_msg);
    }
  }

  void fallback_timer_cb()
  {
    if (!publish_identity_until_odom_) {
      return;
    }

    if (odom_received_) {
      return;
    }

    publish_identity_tf();
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_received_ = true;

    const auto stamp = valid_stamp(msg->header.stamp);

    const auto odom_out = make_relative_odom(*msg, stamp);

    publish_dynamic_tf(stamp, odom_out);

    if (publish_odom_msg_) {
      odom_pub_->publish(odom_out);
    }
  }

  std::string odom_topic_;
  std::string odom_msg_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_map_to_odom_{true};
  bool publish_odom_msg_{true};
  bool publish_identity_until_odom_{true};
  bool use_msg_timestamp_{false};
  bool zero_initial_pose_{true};
  bool odom_received_{false};
  bool origin_initialized_{false};

  double base_z_offset_{0.0};

  tf2::Transform initial_tf_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr fallback_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfOdometryNode>());
  rclcpp::shutdown();
  return 0;
}