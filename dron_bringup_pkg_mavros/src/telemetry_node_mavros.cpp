#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
std::string finite_or_null(double value, int precision = 3)
{
  if (std::isfinite(value)) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }
  return "null";
}
}  // namespace

class TelemetryNode : public rclcpp::Node
{
public:
  TelemetryNode()
  : Node("telemetry_node_mavros")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 5.0);
    pub_ = create_publisher<std_msgs::msg::String>("/manual/telemetry_state", 10);

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&TelemetryNode::state_cb, this, std::placeholders::_1));

    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", 10,
      std::bind(&TelemetryNode::extended_state_cb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", 10,
      std::bind(&TelemetryNode::odom_cb, this, std::placeholders::_1));

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", 10,
      std::bind(&TelemetryNode::battery_cb, this, std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", 10,
      std::bind(&TelemetryNode::gps_cb, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", 10,
      std::bind(&TelemetryNode::imu_cb, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TelemetryNode::loop, this));

    RCLCPP_INFO(get_logger(), "telemetry_node_mavros MAVROS listo. Publicando /manual/telemetry_state");
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"connected\":" << (msg->connected ? "true" : "false")
        << ",\"armed\":" << (msg->armed ? "true" : "false")
        << ",\"guided\":" << (msg->guided ? "true" : "false")
        << ",\"mode\":\"" << msg->mode << "\""
        << ",\"system_status\":" << static_cast<int>(msg->system_status) << "}";
    state_json_ = oss.str();
    has_state_ = true;
  }

  void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"landed_state\":" << static_cast<int>(msg->landed_state)
        << ",\"vtol_state\":" << static_cast<int>(msg->vtol_state) << "}";
    extended_state_json_ = oss.str();
    has_extended_state_ = true;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const auto & v = msg->twist.twist.linear;

    std::ostringstream oss;
    oss << "{\"position_m\":[" << finite_or_null(p.x) << "," << finite_or_null(p.y) << "," << finite_or_null(p.z) << "]"
        << ",\"velocity_m_s\":[" << finite_or_null(v.x) << "," << finite_or_null(v.y) << "," << finite_or_null(v.z) << "]"
        << ",\"q\":[" << finite_or_null(q.w) << "," << finite_or_null(q.x) << "," << finite_or_null(q.y) << "," << finite_or_null(q.z) << "]}";
    odom_json_ = oss.str();
    has_odom_ = true;
  }

  void battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"voltage_v\":" << finite_or_null(msg->voltage)
        << ",\"current_a\":" << finite_or_null(msg->current)
        << ",\"percentage\":" << finite_or_null(msg->percentage) << "}";
    battery_json_ = oss.str();
    has_battery_ = true;
  }

  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"status\":" << static_cast<int>(msg->status.status)
        << ",\"lat_deg\":" << finite_or_null(msg->latitude, 7)
        << ",\"lon_deg\":" << finite_or_null(msg->longitude, 7)
        << ",\"alt_m\":" << finite_or_null(msg->altitude, 3) << "}";
    gps_json_ = oss.str();
    has_gps_ = true;
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & av = msg->angular_velocity;
    const auto & la = msg->linear_acceleration;
    std::ostringstream oss;
    oss << "{\"angular_velocity\":[" << finite_or_null(av.x) << "," << finite_or_null(av.y) << "," << finite_or_null(av.z) << "]"
        << ",\"linear_acceleration\":[" << finite_or_null(la.x) << "," << finite_or_null(la.y) << "," << finite_or_null(la.z) << "]}";
    imu_json_ = oss.str();
    has_imu_ = true;
  }

  void append_field(std::ostringstream & oss, bool & first, const std::string & name, const std::string & value)
  {
    if (!first) oss << ",";
    oss << "\"" << name << "\":" << value;
    first = false;
  }

  void loop()
  {
    std::ostringstream oss;
    bool first = true;
    oss << "{";
    if (has_state_) append_field(oss, first, "mavros_state", state_json_);
    if (has_extended_state_) append_field(oss, first, "extended_state", extended_state_json_);
    if (has_odom_) append_field(oss, first, "odometry", odom_json_);
    if (has_battery_) append_field(oss, first, "battery", battery_json_);
    if (has_gps_) append_field(oss, first, "gps", gps_json_);
    if (has_imu_) append_field(oss, first, "imu", imu_json_);
    oss << "}";

    std_msgs::msg::String msg;
    msg.data = oss.str();
    pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", msg.data.c_str());
  }

  double publish_hz_{5.0};
  bool has_state_{false};
  bool has_extended_state_{false};
  bool has_odom_{false};
  bool has_battery_{false};
  bool has_gps_{false};
  bool has_imu_{false};

  std::string state_json_;
  std::string extended_state_json_;
  std::string odom_json_;
  std::string battery_json_;
  std::string gps_json_;
  std::string imu_json_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
