#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "dron_bringup_pkg/px4_qos.hpp"

class ManualControlNode : public rclcpp::Node
{
public:
  ManualControlNode()
  : Node("manual_control_node")
  {
    loop_hz_ = declare_parameter<double>("loop_hz", 30.0);

    takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude_m", 2.0);
    takeoff_acceptance_m_ = declare_parameter<double>("takeoff_acceptance_m", 0.25);

    require_safety_enable_ = declare_parameter<bool>("require_safety_enable", true);
    auto_arm_on_takeoff_ = declare_parameter<bool>("auto_arm_on_takeoff", false);

    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.8);
    max_z_speed_ = declare_parameter<double>("max_z_speed", 0.4);

    yaw_step_ = declare_parameter<double>("yaw_step", 0.0873);

    max_xy_accel_ = declare_parameter<double>("max_xy_accel", 0.6);
    max_z_accel_ = declare_parameter<double>("max_z_accel", 0.35);

    max_xy_decel_ = declare_parameter<double>("max_xy_decel", 1.2);
    max_z_decel_ = declare_parameter<double>("max_z_decel", 0.8);

    manual_deadman_timeout_s_ = declare_parameter<double>("manual_deadman_timeout_s", 0.30);

    offboard_warmup_cycles_ = declare_parameter<int>("offboard_warmup_cycles", 30);
    auto_arm_delay_cycles_ = declare_parameter<int>("auto_arm_delay_cycles", 10);

    last_manual_cmd_time_ = now();

    offboard_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode",
        dron_bringup_pkg::px4_qos());

    traj_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint",
        dron_bringup_pkg::px4_qos());

    command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        dron_bringup_pkg::px4_qos());

    cmd_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
        "/manual/cmd_vel",
        10,
        std::bind(&ManualControlNode::cmd_cb, this, std::placeholders::_1));

    key_sub_ =
      create_subscription<std_msgs::msg::String>(
        "/manual/key",
        10,
        std::bind(&ManualControlNode::key_cb, this, std::placeholders::_1));

    safety_sub_ =
      create_subscription<std_msgs::msg::Bool>(
        "/manual/safety_enable",
        10,
        std::bind(&ManualControlNode::safety_cb, this, std::placeholders::_1));

    safety_reason_sub_ =
      create_subscription<std_msgs::msg::String>(
        "/manual/safety_reason",
        10,
        std::bind(&ManualControlNode::safety_reason_cb, this, std::placeholders::_1));

    status_sub_ =
      create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v4",
        dron_bringup_pkg::px4_qos(),
        std::bind(&ManualControlNode::status_cb, this, std::placeholders::_1));

    local_pos_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1",
        dron_bringup_pkg::px4_qos(),
        std::bind(&ManualControlNode::local_pos_cb, this, std::placeholders::_1));

    odom_sub_ =
      create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        dron_bringup_pkg::px4_qos(),
        std::bind(&ManualControlNode::odom_cb, this, std::placeholders::_1));

    land_sub_ =
      create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        dron_bringup_pkg::px4_qos(),
        std::bind(&ManualControlNode::land_cb, this, std::placeholders::_1));

    ack_sub_ =
      create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/fmu/out/vehicle_command_ack_v1",
        dron_bringup_pkg::px4_qos(),
        std::bind(&ManualControlNode::ack_cb, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ManualControlNode::loop, this));

    RCLCPP_INFO(
      get_logger(),
      "manual_control_node listo. Control seguro: teclado momentaneo, despegue por posicion, manual por velocidad.");
  }

private:
  enum class ControlState
  {
    IDLE,
    PRE_OFFBOARD,
    TAKEOFF_POSITION,
    MANUAL_VELOCITY,
    LANDING
  };

  static constexpr uint8_t NAVIGATION_STATE_OFFBOARD = 14;
  static constexpr uint8_t ARMING_STATE_ARMED = 2;

  uint64_t now_us()
  {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
  }

  const char * state_name() const
  {
    switch (state_) {
      case ControlState::IDLE:
        return "IDLE";
      case ControlState::PRE_OFFBOARD:
        return "PRE_OFFBOARD";
      case ControlState::TAKEOFF_POSITION:
        return "TAKEOFF_POSITION";
      case ControlState::MANUAL_VELOCITY:
        return "MANUAL_VELOCITY";
      case ControlState::LANDING:
        return "LANDING";
      default:
        return "UNKNOWN";
    }
  }

  static double clamp(double value, double limit)
  {
    if (value > limit) {
      return limit;
    }

    if (value < -limit) {
      return -limit;
    }

    return value;
  }

  static double slew_limit(double current, double target, double max_rate, double dt)
  {
    const double max_delta = max_rate * dt;
    const double delta = target - current;

    if (delta > max_delta) {
      return current + max_delta;
    }

    if (delta < -max_delta) {
      return current - max_delta;
    }

    return target;
  }

  static double wrap_pi(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }

    return angle;
  }

  bool is_offboard() const
  {
    return has_vehicle_status_ &&
      vehicle_status_.nav_state == NAVIGATION_STATE_OFFBOARD;
  }

  bool is_armed() const
  {
    return has_vehicle_status_ &&
      vehicle_status_.arming_state == ARMING_STATE_ARMED;
  }

  bool safety_ok() const
  {
    if (!require_safety_enable_) {
      return true;
    }

    return safety_enabled_;
  }

  bool manual_cmd_timeout() const
  {
    const double elapsed = (now() - last_manual_cmd_time_).seconds();
    return elapsed > manual_deadman_timeout_s_;
  }

  void apply_deadman_timeout()
  {
    if (state_ != ControlState::MANUAL_VELOCITY) {
      return;
    }

    if (manual_cmd_timeout()) {
      latest_cmd_ = geometry_msgs::msg::Twist {};
    }
  }

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;

    latest_cmd_.linear.x = clamp(latest_cmd_.linear.x, max_xy_speed_);
    latest_cmd_.linear.y = clamp(latest_cmd_.linear.y, max_xy_speed_);
    latest_cmd_.linear.z = clamp(latest_cmd_.linear.z, max_z_speed_);

    latest_cmd_.angular.z = 0.0;

    last_manual_cmd_time_ = now();
  }

  void key_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }

    const char key = msg->data[0];

    switch (key) {
      case 'b':
        prepare_offboard();
        break;

      case 'm':
        arm();
        break;

      case 'n':
        disarm();
        break;

      case 't':
        start_takeoff();
        break;

      case 'l':
        land();
        break;

      case 'q':
        yaw_left_step();
        break;

      case 'e':
        yaw_right_step();
        break;

      case ' ':
      case 'z':
        stop_motion_immediately();
        break;

      case 'p':
        print_state();
        break;

      default:
        break;
    }
  }

  void safety_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    safety_enabled_ = msg->data;
  }

  void safety_reason_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    safety_reason_ = msg->data;
  }

  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
    has_vehicle_status_ = true;
  }

  void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;

    has_local_position_ = msg->xy_valid && msg->z_valid;

    if (!setpoint_initialized_ && has_local_position_) {
      hold_x_ = current_x_;
      hold_y_ = current_y_;
      hold_z_ = current_z_;
      setpoint_initialized_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Setpoint inicializado: x=%.2f y=%.2f z=%.2f",
        hold_x_,
        hold_y_,
        hold_z_);
    }
  }

  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    const auto & q = msg->q;

    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    current_yaw_ =
      std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));

    if (!yaw_initialized_) {
      hold_yaw_ = static_cast<float>(current_yaw_);
      yaw_initialized_ = true;
    }
  }

  void land_cb(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    landed_ = msg->landed;
    has_land_detected_ = true;
  }

  void ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {
    const bool accepted =
      msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;

    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
      offboard_accepted_ = accepted;
    }

    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
      arm_accepted_ = accepted;
    }

    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND) {
      land_accepted_ = accepted;
    }

    RCLCPP_INFO(
      get_logger(),
      "ACK command=%u result=%u accepted=%s",
      msg->command,
      msg->result,
      accepted ? "true" : "false");
  }

  void ensure_hold_setpoint_initialized()
  {
    if (!setpoint_initialized_) {
      hold_x_ = has_local_position_ ? current_x_ : 0.0f;
      hold_y_ = has_local_position_ ? current_y_ : 0.0f;
      hold_z_ = has_local_position_ ? current_z_ : 0.0f;
      setpoint_initialized_ = true;
    }

    // if (!yaw_initialized_) {
    //   hold_yaw_ = static_cast<float>(current_yaw_);
    //   yaw_initialized_ = true;
    // }
  }

  void publish_vehicle_command(
    uint16_t command,
    float p1 = 0.0F,
    float p2 = 0.0F,
    float p3 = 0.0F,
    float p4 = 0.0F,
    float p5 = 0.0F,
    float p6 = 0.0F,
    float p7 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg {};

    msg.timestamp = now_us();

    msg.param1 = p1;
    msg.param2 = p2;
    msg.param3 = p3;
    msg.param4 = p4;
    msg.param5 = p5;
    msg.param6 = p6;
    msg.param7 = p7;

    msg.command = command;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    command_pub_->publish(msg);
  }

  void arm()
  {
    if (!safety_ok()) {
      RCLCPP_WARN(
        get_logger(),
        "ARM bloqueado por safety: %s",
        safety_reason_.c_str());
      return;
    }

    if (has_vehicle_status_ && !vehicle_status_.pre_flight_checks_pass) {
      RCLCPP_WARN(
        get_logger(),
        "ARM enviado aunque pre_flight_checks_pass=false. PX4 puede rechazarlo.");
    }

    arm_accepted_ = false;

    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0F);

    RCLCPP_INFO(get_logger(), "Comando ARM enviado");
  }

  void disarm()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0F);

    state_ = ControlState::IDLE;

    stop_motion_immediately();

    offboard_requested_ = false;
    offboard_command_sent_ = false;
    auto_arm_after_offboard_ = false;

    offboard_accepted_ = false;
    arm_accepted_ = false;
    land_accepted_ = false;

    RCLCPP_WARN(get_logger(), "Comando DISARM enviado");
  }

  void prepare_offboard()
  {
    ensure_hold_setpoint_initialized();

    state_ = ControlState::PRE_OFFBOARD;

    stop_motion_immediately();

    offboard_requested_ = true;
    offboard_command_sent_ = false;
    auto_arm_after_offboard_ = false;
    offboard_accepted_ = false;

    offboard_warmup_counter_ = 0;
    auto_arm_delay_counter_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "Preparando Offboard. Publicando setpoint de posicion antes de pedir Offboard.");
  }

  void start_takeoff()
  {
    if (!safety_ok()) {
      RCLCPP_WARN(
        get_logger(),
        "TAKEOFF bloqueado por safety: %s",
        safety_reason_.c_str());
      return;
    }

    ensure_hold_setpoint_initialized();

    stop_motion_immediately();

    hold_x_ = has_local_position_ ? current_x_ : hold_x_;
    hold_y_ = has_local_position_ ? current_y_ : hold_y_;

    const float base_z = has_local_position_ ? current_z_ : hold_z_;
    takeoff_z_ = base_z - static_cast<float>(takeoff_altitude_m_);
    hold_z_ = takeoff_z_;

    state_ = ControlState::TAKEOFF_POSITION;

    if (!is_offboard()) {
      offboard_requested_ = true;
      offboard_command_sent_ = false;
      offboard_warmup_counter_ = 0;
      offboard_accepted_ = false;
    }

    if (auto_arm_on_takeoff_ && !is_armed()) {
      auto_arm_after_offboard_ = true;
      auto_arm_delay_counter_ = 0;
    }

    RCLCPP_INFO(
      get_logger(),
      "TAKEOFF_POSITION: objetivo x=%.2f y=%.2f z=%.2f",
      hold_x_,
      hold_y_,
      hold_z_);
  }

  void land()
  {
    state_ = ControlState::LANDING;

    stop_motion_immediately();

    land_accepted_ = false;

    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_WARN(get_logger(), "Comando LAND enviado. Dejando de publicar setpoints Offboard.");
  }

  void send_offboard_mode_command()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F,
      6.0F);

    offboard_command_sent_ = true;
    offboard_requested_ = false;

    RCLCPP_INFO(get_logger(), "Comando OFFBOARD enviado");
  }

  void stop_motion_immediately()
  {
    latest_cmd_ = geometry_msgs::msg::Twist {};
    smoothed_cmd_ = geometry_msgs::msg::Twist {};
    last_manual_cmd_time_ = now();
  }

  void yaw_left_step()
  {
    if (state_ != ControlState::MANUAL_VELOCITY &&
        state_ != ControlState::TAKEOFF_POSITION) {
      return;
    }

    ensure_hold_setpoint_initialized();

    hold_yaw_ = static_cast<float>(
      wrap_pi(static_cast<double>(hold_yaw_) + yaw_step_));

    RCLCPP_INFO(get_logger(), "Yaw izquierda: hold_yaw=%.2f rad", hold_yaw_);
  }

  void yaw_right_step()
  {
    if (state_ != ControlState::MANUAL_VELOCITY &&
        state_ != ControlState::TAKEOFF_POSITION) {
      return;
    }

    ensure_hold_setpoint_initialized();

    hold_yaw_ = static_cast<float>(
      wrap_pi(static_cast<double>(hold_yaw_) - yaw_step_));

    RCLCPP_INFO(get_logger(), "Yaw derecha: hold_yaw=%.2f rad", hold_yaw_);
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg {};

    msg.timestamp = now_us();

    if (state_ == ControlState::MANUAL_VELOCITY) {
      msg.position = false;
      msg.velocity = true;
    } else {
      msg.position = true;
      msg.velocity = false;
    }

    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_pub_->publish(msg);
  }

  void publish_position_setpoint()
  {
    ensure_hold_setpoint_initialized();

    px4_msgs::msg::TrajectorySetpoint msg {};

    msg.timestamp = now_us();

    msg.position = {hold_x_, hold_y_, hold_z_};
    msg.yaw = hold_yaw_;

    traj_pub_->publish(msg);
  }

  void update_smoothed_cmd()
  {
    const double dt = 1.0 / loop_hz_;

    const double xy_rate_x =
      std::fabs(latest_cmd_.linear.x) < std::fabs(smoothed_cmd_.linear.x)
        ? max_xy_decel_
        : max_xy_accel_;

    const double xy_rate_y =
      std::fabs(latest_cmd_.linear.y) < std::fabs(smoothed_cmd_.linear.y)
        ? max_xy_decel_
        : max_xy_accel_;

    const double z_rate =
      std::fabs(latest_cmd_.linear.z) < std::fabs(smoothed_cmd_.linear.z)
        ? max_z_decel_
        : max_z_accel_;

    smoothed_cmd_.linear.x =
      slew_limit(smoothed_cmd_.linear.x, latest_cmd_.linear.x, xy_rate_x, dt);

    smoothed_cmd_.linear.y =
      slew_limit(smoothed_cmd_.linear.y, latest_cmd_.linear.y, xy_rate_y, dt);

    smoothed_cmd_.linear.z =
      slew_limit(smoothed_cmd_.linear.z, latest_cmd_.linear.z, z_rate, dt);

    smoothed_cmd_.angular.z = 0.0;
  }

  void publish_velocity_setpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg {};

    const float nan = std::numeric_limits<float>::quiet_NaN();

    update_smoothed_cmd();

    const double forward = smoothed_cmd_.linear.x;
    const double left = smoothed_cmd_.linear.y;
    const double up = smoothed_cmd_.linear.z;

    const double c = std::cos(current_yaw_);
    const double s = std::sin(current_yaw_);

    const double north = c * forward + s * left;
    const double east = s * forward - c * left;
    const double down = -up;

    msg.timestamp = now_us();

    msg.position = {nan, nan, nan};

    msg.velocity = {
      static_cast<float>(north),
      static_cast<float>(east),
      static_cast<float>(down)
    };

    msg.acceleration = {nan, nan, nan};
    msg.jerk = {nan, nan, nan};

    msg.yaw = hold_yaw_;
    msg.yawspeed = nan;

    traj_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    if (state_ == ControlState::MANUAL_VELOCITY) {
      publish_velocity_setpoint();
    } else {
      publish_position_setpoint();
    }
  }

  void update_takeoff_transition()
  {
    if (state_ != ControlState::TAKEOFF_POSITION) {
      return;
    }

    if (!has_local_position_) {
      return;
    }

    const double error_z =
      std::fabs(static_cast<double>(current_z_ - takeoff_z_));

    if (error_z <= takeoff_acceptance_m_) {
      if (is_offboard() && is_armed()) {
        stop_motion_immediately();

        state_ = ControlState::MANUAL_VELOCITY;

        RCLCPP_INFO(
          get_logger(),
          "Altura alcanzada. Cambiando a MANUAL_VELOCITY.");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Altura alcanzada, pero no paso a MANUAL: offboard=%s armed=%s",
          is_offboard() ? "true" : "false",
          is_armed() ? "true" : "false");
      }
    }
  }

  void update_offboard_request()
  {
    if (!offboard_requested_) {
      return;
    }

    if (offboard_warmup_counter_ < offboard_warmup_cycles_) {
      ++offboard_warmup_counter_;
      return;
    }

    send_offboard_mode_command();
  }

  void update_auto_arm()
  {
    if (!auto_arm_after_offboard_) {
      return;
    }

    if (!offboard_command_sent_ && !is_offboard()) {
      return;
    }

    if (auto_arm_delay_counter_ < auto_arm_delay_cycles_) {
      ++auto_arm_delay_counter_;
      return;
    }

    auto_arm_after_offboard_ = false;

    if (!is_armed()) {
      arm();
    }
  }

  void loop()
  {
    if (state_ == ControlState::LANDING) {
      stop_motion_immediately();

      if (has_land_detected_ && landed_) {
        state_ = ControlState::IDLE;
        RCLCPP_INFO(get_logger(), "Aterrizaje detectado. Estado IDLE.");
      }

      return;
    }

    apply_deadman_timeout();

    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    update_offboard_request();
    update_auto_arm();
    update_takeoff_transition();

    if (require_safety_enable_ && !safety_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Safety no OK: %s",
        safety_reason_.c_str());
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,
      "state=%s nav=%u arm=%u offboard=%s armed=%s local=%s pos=[%.2f %.2f %.2f] hold=[%.2f %.2f %.2f %.2f] cmd=[%.2f %.2f %.2f] smooth=[%.2f %.2f %.2f]",
      state_name(),
      has_vehicle_status_ ? vehicle_status_.nav_state : 255,
      has_vehicle_status_ ? vehicle_status_.arming_state : 255,
      is_offboard() ? "true" : "false",
      is_armed() ? "true" : "false",
      has_local_position_ ? "true" : "false",
      current_x_,
      current_y_,
      current_z_,
      hold_x_,
      hold_y_,
      hold_z_,
      hold_yaw_,
      latest_cmd_.linear.x,
      latest_cmd_.linear.y,
      latest_cmd_.linear.z,
      smoothed_cmd_.linear.x,
      smoothed_cmd_.linear.y,
      smoothed_cmd_.linear.z);
  }

  void print_state()
  {
    RCLCPP_INFO(
      get_logger(),
      "PX4 state=%s nav=%u arm=%u offboard=%s armed=%s failsafe=%s preflight=%s landed=%s | ACK offboard=%s arm=%s land=%s | current x=%.2f y=%.2f z=%.2f yaw=%.2f | hold x=%.2f y=%.2f z=%.2f yaw=%.2f",
      state_name(),
      has_vehicle_status_ ? vehicle_status_.nav_state : 255,
      has_vehicle_status_ ? vehicle_status_.arming_state : 255,
      is_offboard() ? "true" : "false",
      is_armed() ? "true" : "false",
      (has_vehicle_status_ && vehicle_status_.failsafe) ? "true" : "false",
      (has_vehicle_status_ && vehicle_status_.pre_flight_checks_pass) ? "true" : "false",
      landed_ ? "true" : "false",
      offboard_accepted_ ? "true" : "false",
      arm_accepted_ ? "true" : "false",
      land_accepted_ ? "true" : "false",
      current_x_,
      current_y_,
      current_z_,
      current_yaw_,
      hold_x_,
      hold_y_,
      hold_z_,
      hold_yaw_);
  }

  double loop_hz_{30.0};

  double takeoff_altitude_m_{2.0};
  double takeoff_acceptance_m_{0.25};

  bool require_safety_enable_{true};
  bool auto_arm_on_takeoff_{false};

  double max_xy_speed_{0.8};
  double max_z_speed_{0.4};

  double yaw_step_{0.0873};

  double max_xy_accel_{0.6};
  double max_z_accel_{0.35};

  double max_xy_decel_{1.2};
  double max_z_decel_{0.8};

  double manual_deadman_timeout_s_{0.30};

  int offboard_warmup_cycles_{30};
  int auto_arm_delay_cycles_{10};

  ControlState state_{ControlState::IDLE};

  bool safety_enabled_{false};
  std::string safety_reason_{"sin datos de safety todavia"};

  bool offboard_requested_{false};
  bool offboard_command_sent_{false};
  bool auto_arm_after_offboard_{false};

  bool offboard_accepted_{false};
  bool arm_accepted_{false};
  bool land_accepted_{false};

  int offboard_warmup_counter_{0};
  int auto_arm_delay_counter_{0};

  bool has_vehicle_status_{false};
  bool has_local_position_{false};
  bool has_land_detected_{false};
  bool landed_{true};

  bool setpoint_initialized_{false};
  bool yaw_initialized_{false};

  float current_x_{0.0f};
  float current_y_{0.0f};
  float current_z_{0.0f};
  double current_yaw_{0.0};

  float hold_x_{0.0f};
  float hold_y_{0.0f};
  float hold_z_{0.0f};
  float hold_yaw_{0.0f};

  float takeoff_z_{-2.0f};

  geometry_msgs::msg::Twist latest_cmd_ {};
  geometry_msgs::msg::Twist smoothed_cmd_ {};

  rclcpp::Time last_manual_cmd_time_;

  px4_msgs::msg::VehicleStatus vehicle_status_ {};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_reason_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}