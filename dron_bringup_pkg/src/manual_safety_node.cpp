#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "dron_bringup_pkg/px4_qos.hpp"

class ManualSafetyNode : public rclcpp::Node
{
public:
  ManualSafetyNode()
  : Node("manual_safety_node")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
    min_battery_remaining_ = declare_parameter<double>("min_battery_remaining", 0.20);
    block_if_failsafe_ = declare_parameter<bool>("block_if_failsafe", true);
    block_if_landed_ = declare_parameter<bool>("block_if_landed", false);

    enable_pub_ = create_publisher<std_msgs::msg::Bool>("/manual/safety_enable", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/manual/safety_reason", 10);

    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>("/fmu/out/battery_status_v1", dron_bringup_pkg::px4_qos(), std::bind(&ManualSafetyNode::battery_cb, this, std::placeholders::_1));
    failsafe_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>("/fmu/out/failsafe_flags", dron_bringup_pkg::px4_qos(), std::bind(&ManualSafetyNode::failsafe_cb, this, std::placeholders::_1));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v4", dron_bringup_pkg::px4_qos(), std::bind(&ManualSafetyNode::status_cb, this, std::placeholders::_1));
    land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", dron_bringup_pkg::px4_qos(), std::bind(&ManualSafetyNode::land_cb, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&ManualSafetyNode::loop, this));
    RCLCPP_INFO(get_logger(), "manual_safety_node listo");
  }

private:
  void battery_cb(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    battery_ = *msg;
    has_battery_ = true;
  }

  void failsafe_cb(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
  {
    (void)msg;
    has_failsafe_flags_ = true;
  }

  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    status_ = *msg;
    has_status_ = true;
  }

  void land_cb(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    land_ = *msg;
    has_land_ = true;
  }

  void add_reason(std::ostringstream & oss, bool & first, const std::string & reason)
  {
    if (!first) {
      oss << "; ";
    }
    oss << reason;
    first = false;
  }

  void loop()
  {
    bool enabled = true;
    bool first_reason = true;
    std::ostringstream reasons;

    if (!has_battery_) {
      add_reason(reasons, first_reason, "sin BatteryStatus");
    } else if (battery_.remaining >= 0.0F && battery_.remaining < min_battery_remaining_) {
      enabled = false;
      std::ostringstream r;
      r << "bateria baja: " << static_cast<int>(battery_.remaining * 100.0F) << "%";
      add_reason(reasons, first_reason, r.str());
    }

    if (block_if_failsafe_) {
      if (has_status_ && status_.failsafe) {
        enabled = false;
        add_reason(reasons, first_reason, "failsafe activo en VehicleStatus");
      } else if (!has_failsafe_flags_) {
        add_reason(reasons, first_reason, "sin FailsafeFlags");
      }
    }

    if (block_if_landed_ && has_land_ && land_.landed) {
      enabled = false;
      add_reason(reasons, first_reason, "vehiculo detectado en tierra");
    }

    if (!has_status_) {
      add_reason(reasons, first_reason, "sin VehicleStatus");
    }

    std_msgs::msg::Bool enable_msg;
    enable_msg.data = enabled;
    enable_pub_->publish(enable_msg);

    std_msgs::msg::String reason_msg;
    reason_msg.data = enabled ? "OK" : reasons.str();
    reason_pub_->publish(reason_msg);
  }

  double publish_hz_{10.0};
  double min_battery_remaining_{0.20};
  bool block_if_failsafe_{true};
  bool block_if_landed_{false};
  bool has_battery_{false};
  bool has_failsafe_flags_{false};
  bool has_status_{false};
  bool has_land_{false};

  px4_msgs::msg::BatteryStatus battery_{};
  px4_msgs::msg::VehicleStatus status_{};
  px4_msgs::msg::VehicleLandDetected land_{};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
