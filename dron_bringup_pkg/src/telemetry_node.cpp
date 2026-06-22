#include <array>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "dron_bringup_pkg/px4_qos.hpp"

namespace
{
std::string finite_or_null(float value)
{
  if (std::isfinite(value)) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << value;
    return oss.str();
  }
  return "null";
}

std::string finite_or_null(double value)
{
  if (std::isfinite(value)) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(7) << value;
    return oss.str();
  }
  return "null";
}

template<typename T, std::size_t N>
std::string array_json(const std::array<T, N> & values)
{
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < N; ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << finite_or_null(static_cast<float>(values[i]));
  }
  oss << "]";
  return oss.str();
}
}  // namespace

class TelemetryNode : public rclcpp::Node
{
public:
  TelemetryNode()
  : Node("telemetry_node")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 5.0);
    pub_ = create_publisher<std_msgs::msg::String>("/manual/telemetry_state", 10);

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v4", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::status_cb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::odom_cb, this, std::placeholders::_1));
    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::local_pos_cb, this, std::placeholders::_1));
    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>("/fmu/out/battery_status_v1", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::battery_cb, this, std::placeholders::_1));
    failsafe_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>("/fmu/out/failsafe_flags", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::failsafe_cb, this, std::placeholders::_1));
    land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::land_cb, this, std::placeholders::_1));
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", dron_bringup_pkg::px4_qos(), std::bind(&TelemetryNode::gps_cb, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&TelemetryNode::loop, this));
    RCLCPP_INFO(get_logger(), "telemetry_node listo. Publicando /manual/telemetry_state");
  }

private:
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"arming_state\":" << static_cast<int>(msg->arming_state)
        << ",\"nav_state\":" << static_cast<int>(msg->nav_state)
        << ",\"hil_state\":" << static_cast<int>(msg->hil_state)
        << ",\"failsafe\":" << (msg->failsafe ? "true" : "false") << "}";
    vehicle_status_json_ = oss.str();
    has_vehicle_status_ = true;
  }

  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"position_m\":" << array_json(msg->position)
        << ",\"velocity_m_s\":" << array_json(msg->velocity)
        << ",\"q\":" << array_json(msg->q) << "}";
    odom_json_ = oss.str();
    has_odom_ = true;
  }

  void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"xy_valid\":" << (msg->xy_valid ? "true" : "false")
        << ",\"z_valid\":" << (msg->z_valid ? "true" : "false")
        << ",\"v_xy_valid\":" << (msg->v_xy_valid ? "true" : "false")
        << ",\"v_z_valid\":" << (msg->v_z_valid ? "true" : "false")
        << ",\"x\":" << finite_or_null(msg->x)
        << ",\"y\":" << finite_or_null(msg->y)
        << ",\"z\":" << finite_or_null(msg->z)
        << ",\"vx\":" << finite_or_null(msg->vx)
        << ",\"vy\":" << finite_or_null(msg->vy)
        << ",\"vz\":" << finite_or_null(msg->vz) << "}";
    local_pos_json_ = oss.str();
    has_local_pos_ = true;
  }

  void battery_cb(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"voltage_v\":" << finite_or_null(msg->voltage_v)
        << ",\"current_a\":" << finite_or_null(msg->current_a)
        << ",\"remaining\":" << finite_or_null(msg->remaining) << "}";
    battery_json_ = oss.str();
    has_battery_ = true;
  }

  void failsafe_cb(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
  {
    (void)msg;
    failsafe_json_ = "{\"received\":true}";
    has_failsafe_ = true;
  }

  void land_cb(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"landed\":" << (msg->landed ? "true" : "false")
        << ",\"freefall\":" << (msg->freefall ? "true" : "false")
        << ",\"ground_contact\":" << (msg->ground_contact ? "true" : "false") << "}";
    land_json_ = oss.str();
    has_land_ = true;
  }

  void gps_cb(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{\"fix_type\":" << static_cast<int>(msg->fix_type)
        << ",\"lat_deg\":" << finite_or_null(msg->latitude_deg)
        << ",\"lon_deg\":" << finite_or_null(msg->longitude_deg)
        << ",\"alt_m\":" << finite_or_null(msg->altitude_msl_m)
        << ",\"satellites_used\":" << static_cast<int>(msg->satellites_used) << "}";
    gps_json_ = oss.str();
    has_gps_ = true;
  }

  void append_field(std::ostringstream & oss, bool & first, const std::string & name, const std::string & value)
  {
    if (!first) {
      oss << ",";
    }
    oss << "\"" << name << "\":" << value;
    first = false;
  }

  void loop()
  {
    std::ostringstream oss;
    bool first = true;
    oss << "{";
    if (has_vehicle_status_) append_field(oss, first, "vehicle_status", vehicle_status_json_);
    if (has_odom_) append_field(oss, first, "odometry", odom_json_);
    if (has_local_pos_) append_field(oss, first, "local_position", local_pos_json_);
    if (has_battery_) append_field(oss, first, "battery", battery_json_);
    if (has_failsafe_) append_field(oss, first, "failsafe", failsafe_json_);
    if (has_land_) append_field(oss, first, "land", land_json_);
    if (has_gps_) append_field(oss, first, "gps", gps_json_);
    oss << "}";

    std_msgs::msg::String msg;
    msg.data = oss.str();
    pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", msg.data.c_str());
  }

  double publish_hz_{5.0};
  bool has_vehicle_status_{false};
  bool has_odom_{false};
  bool has_local_pos_{false};
  bool has_battery_{false};
  bool has_failsafe_{false};
  bool has_land_{false};
  bool has_gps_{false};
  std::string vehicle_status_json_;
  std::string odom_json_;
  std::string local_pos_json_;
  std::string battery_json_;
  std::string failsafe_json_;
  std::string land_json_;
  std::string gps_json_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
