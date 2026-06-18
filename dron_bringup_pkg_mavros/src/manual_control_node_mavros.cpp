#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class ManualControlNode : public rclcpp::Node
{
public:
  ManualControlNode()
  : Node("manual_control_node_mavros")
  {
    loop_hz_ = declare_parameter<double>("loop_hz", 30.0);

    takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude_m", 2.0);
    takeoff_acceptance_m_ = declare_parameter<double>("takeoff_acceptance_m", 0.25);

    require_safety_enable_ = declare_parameter<bool>("require_safety_enable", true);
    auto_arm_on_takeoff_ = declare_parameter<bool>("auto_arm_on_takeoff", false);

    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.8);
    max_z_speed_ = declare_parameter<double>("max_z_speed", 0.4);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.6);

    max_xy_accel_ = declare_parameter<double>("max_xy_accel", 0.6);
    max_z_accel_ = declare_parameter<double>("max_z_accel", 0.35);
    max_yaw_accel_ = declare_parameter<double>("max_yaw_accel", 1.0);

    max_xy_decel_ = declare_parameter<double>("max_xy_decel", 1.2);
    max_z_decel_ = declare_parameter<double>("max_z_decel", 0.8);
    max_yaw_decel_ = declare_parameter<double>("max_yaw_decel", 1.5);

    manual_deadman_timeout_s_ = declare_parameter<double>("manual_deadman_timeout_s", 0.30);
    offboard_warmup_cycles_ = declare_parameter<int>("offboard_warmup_cycles", 30);
    auto_arm_delay_cycles_ = declare_parameter<int>("auto_arm_delay_cycles", 10);

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/manual/cmd_vel", 10,
      std::bind(&ManualControlNode::cmd_cb, this, std::placeholders::_1));

    key_sub_ = create_subscription<std_msgs::msg::String>(
      "/manual/key", 10,
      std::bind(&ManualControlNode::key_cb, this, std::placeholders::_1));

    safety_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/manual/safety_enable", 10,
      std::bind(&ManualControlNode::safety_cb, this, std::placeholders::_1));

    safety_reason_sub_ = create_subscription<std_msgs::msg::String>(
      "/manual/safety_reason", 10,
      std::bind(&ManualControlNode::safety_reason_cb, this, std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&ManualControlNode::state_cb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", 10,
      std::bind(&ManualControlNode::odom_cb, this, std::placeholders::_1));

    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", 10,
      std::bind(&ManualControlNode::extended_state_cb, this, std::placeholders::_1));

    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    last_manual_cmd_time_ = now();

    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ManualControlNode::loop, this));

    RCLCPP_INFO(
      get_logger(),
      "manual_control_node_mavros MAVROS listo. Publica /mavros/setpoint_raw/local y usa servicios /mavros/set_mode + /mavros/cmd/arming.");
  }

private:
  enum class ControlState
  {
    IDLE,
    PRE_OFFBOARD,
    TAKEOFF_VELOCITY,
    MANUAL_VELOCITY,
    LANDING
  };

  const char * state_name() const
  {
    switch (state_) {
      case ControlState::IDLE: return "IDLE";
      case ControlState::PRE_OFFBOARD: return "PRE_OFFBOARD";
      case ControlState::TAKEOFF_VELOCITY: return "TAKEOFF_VELOCITY";
      case ControlState::MANUAL_VELOCITY: return "MANUAL_VELOCITY";
      case ControlState::LANDING: return "LANDING";
      default: return "UNKNOWN";
    }
  }

  static double clamp(double value, double limit)
  {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
  }

  static double slew_limit(double current, double target, double max_rate, double dt)
  {
    const double max_delta = max_rate * dt;
    const double delta = target - current;

    if (delta > max_delta) return current + max_delta;
    if (delta < -max_delta) return current - max_delta;
    return target;
  }

  bool is_connected() const
  {
    return has_mavros_state_ && mavros_state_.connected;
  }

  bool is_offboard() const
  {
    return has_mavros_state_ && mavros_state_.mode == "OFFBOARD";
  }

  bool is_armed() const
  {
    return has_mavros_state_ && mavros_state_.armed;
  }

  bool safety_ok() const
  {
    if (!require_safety_enable_) return true;
    return safety_enabled_;
  }

  bool manual_cmd_timeout() const
  {
    return (now() - last_manual_cmd_time_).seconds() > manual_deadman_timeout_s_;
  }

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;

    latest_cmd_.linear.x = clamp(latest_cmd_.linear.x, max_xy_speed_);
    latest_cmd_.linear.y = clamp(latest_cmd_.linear.y, max_xy_speed_);
    latest_cmd_.linear.z = clamp(latest_cmd_.linear.z, max_z_speed_);
    latest_cmd_.angular.z = clamp(latest_cmd_.angular.z, max_yaw_rate_);

    last_manual_cmd_time_ = now();
  }

  void key_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) return;

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

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    mavros_state_ = *msg;
    has_mavros_state_ = true;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    has_local_position_ = true;

    const auto & q_msg = msg->pose.pose.orientation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
  }

  void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    landed_ = msg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
    has_extended_state_ = true;
  }

  void call_set_mode(const std::string & mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Servicio /mavros/set_mode no disponible");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;
    req->custom_mode = mode;

    set_mode_client_->async_send_request(
      req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        const auto res = future.get();
        RCLCPP_INFO(
          get_logger(),
          "SET_MODE %s enviado: mode_sent=%s",
          mode.c_str(),
          res->mode_sent ? "true" : "false");
      });
  }

  void call_arm(bool value)
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Servicio /mavros/cmd/arming no disponible");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = value;

    arming_client_->async_send_request(
      req,
      [this, value](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        const auto res = future.get();
        RCLCPP_INFO(
          get_logger(),
          "%s enviado: success=%s result=%u",
          value ? "ARM" : "DISARM",
          res->success ? "true" : "false",
          res->result);
      });
  }

  void prepare_offboard()
  {
    if (!is_connected()) {
      RCLCPP_WARN(get_logger(), "No preparo OFFBOARD: MAVROS no está conectado a la FCU");
      return;
    }

    state_ = ControlState::PRE_OFFBOARD;
    stop_motion_immediately();
    offboard_warmup_counter_ = 0;
    offboard_command_sent_ = false;

    RCLCPP_INFO(get_logger(), "Preparando OFFBOARD con setpoints de velocidad cero.");
  }

  void arm()
  {
    if (!safety_ok()) {
      RCLCPP_WARN(get_logger(), "ARM bloqueado por safety: %s", safety_reason_.c_str());
      return;
    }

    call_arm(true);
  }

  void disarm()
  {
    call_arm(false);
    state_ = ControlState::IDLE;
    stop_motion_immediately();
    auto_arm_after_offboard_ = false;
    offboard_command_sent_ = false;
    RCLCPP_WARN(get_logger(), "DISARM solicitado");
  }

  void start_takeoff()
  {
    if (!safety_ok()) {
      RCLCPP_WARN(get_logger(), "TAKEOFF bloqueado por safety: %s", safety_reason_.c_str());
      return;
    }

    if (!has_local_position_) {
      RCLCPP_WARN(get_logger(), "TAKEOFF bloqueado: todavía no tengo /mavros/local_position/odom");
      return;
    }

    takeoff_target_z_ = current_z_ + takeoff_altitude_m_;
    state_ = ControlState::TAKEOFF_VELOCITY;
    stop_motion_immediately();

    if (!is_offboard()) {
      offboard_warmup_counter_ = 0;
      offboard_command_sent_ = false;
    }

    if (auto_arm_on_takeoff_ && !is_armed()) {
      auto_arm_after_offboard_ = true;
      auto_arm_delay_counter_ = 0;
    }

    RCLCPP_INFO(
      get_logger(),
      "TAKEOFF_VELOCITY: z actual=%.2f m, objetivo=%.2f m",
      current_z_,
      takeoff_target_z_);
  }

  void land()
  {
    state_ = ControlState::LANDING;
    stop_motion_immediately();
    call_set_mode("AUTO.LAND");
    RCLCPP_WARN(get_logger(), "LAND solicitado con modo AUTO.LAND");
  }

  void stop_motion_immediately()
  {
    latest_cmd_ = geometry_msgs::msg::Twist {};
    smoothed_cmd_ = geometry_msgs::msg::Twist {};
    last_manual_cmd_time_ = now();
  }

  void update_smoothed_cmd()
  {
    const double dt = 1.0 / loop_hz_;

    const double xy_rate_x =
      std::fabs(latest_cmd_.linear.x) < std::fabs(smoothed_cmd_.linear.x) ?
      max_xy_decel_ : max_xy_accel_;

    const double xy_rate_y =
      std::fabs(latest_cmd_.linear.y) < std::fabs(smoothed_cmd_.linear.y) ?
      max_xy_decel_ : max_xy_accel_;

    const double z_rate =
      std::fabs(latest_cmd_.linear.z) < std::fabs(smoothed_cmd_.linear.z) ?
      max_z_decel_ : max_z_accel_;

    const double yaw_rate =
      std::fabs(latest_cmd_.angular.z) < std::fabs(smoothed_cmd_.angular.z) ?
      max_yaw_decel_ : max_yaw_accel_;

    smoothed_cmd_.linear.x = slew_limit(smoothed_cmd_.linear.x, latest_cmd_.linear.x, xy_rate_x, dt);
    smoothed_cmd_.linear.y = slew_limit(smoothed_cmd_.linear.y, latest_cmd_.linear.y, xy_rate_y, dt);
    smoothed_cmd_.linear.z = slew_limit(smoothed_cmd_.linear.z, latest_cmd_.linear.z, z_rate, dt);
    smoothed_cmd_.angular.z = slew_limit(smoothed_cmd_.angular.z, latest_cmd_.angular.z, yaw_rate, dt);
  }

  void publish_body_velocity_setpoint(double forward, double left, double up, double yaw_rate_left)
  {
    mavros_msgs::msg::PositionTarget sp {};

    sp.header.stamp = now();
    sp.header.frame_id = "base_link";
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;

    sp.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_PX |
      mavros_msgs::msg::PositionTarget::IGNORE_PY |
      mavros_msgs::msg::PositionTarget::IGNORE_PZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW;

    // FRAME_BODY_NED: x=forward, y=right, z=down.
    // Teclado:       x=forward, y=left,  z=up.
    sp.velocity.x = forward;
    sp.velocity.y = -left;
    sp.velocity.z = -up;

    // En NED, el signo de yaw es el contrario al habitual ENU/ROS.
    sp.yaw_rate = -yaw_rate_left;

    setpoint_pub_->publish(sp);
  }

  void publish_current_setpoint()
  {
    if (state_ == ControlState::TAKEOFF_VELOCITY) {
      const double remaining = takeoff_target_z_ - current_z_;
      const double up = remaining > takeoff_acceptance_m_ ? max_z_speed_ : 0.0;
      publish_body_velocity_setpoint(0.0, 0.0, up, 0.0);
      return;
    }

    if (state_ == ControlState::MANUAL_VELOCITY) {
      update_smoothed_cmd();
      publish_body_velocity_setpoint(
        smoothed_cmd_.linear.x,
        smoothed_cmd_.linear.y,
        smoothed_cmd_.linear.z,
        smoothed_cmd_.angular.z);
      return;
    }

    publish_body_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
  }

  void update_offboard_request()
  {
    if (state_ != ControlState::PRE_OFFBOARD && state_ != ControlState::TAKEOFF_VELOCITY) {
      return;
    }

    if (is_offboard()) {
      if (state_ == ControlState::PRE_OFFBOARD) {
        state_ = ControlState::MANUAL_VELOCITY;
        RCLCPP_INFO(get_logger(), "OFFBOARD activo. Entrando en MANUAL_VELOCITY.");
      }
      return;
    }

    if (offboard_command_sent_) {
      return;
    }

    if (offboard_warmup_counter_ < offboard_warmup_cycles_) {
      ++offboard_warmup_counter_;
      return;
    }

    call_set_mode("OFFBOARD");
    offboard_command_sent_ = true;
  }

  void update_auto_arm()
  {
    if (!auto_arm_after_offboard_) return;
    if (!is_offboard()) return;

    if (auto_arm_delay_counter_ < auto_arm_delay_cycles_) {
      ++auto_arm_delay_counter_;
      return;
    }

    auto_arm_after_offboard_ = false;

    if (!is_armed()) {
      arm();
    }
  }

  void update_takeoff_transition()
  {
    if (state_ != ControlState::TAKEOFF_VELOCITY || !has_local_position_) return;

    if ((takeoff_target_z_ - current_z_) <= takeoff_acceptance_m_) {
      stop_motion_immediately();
      state_ = ControlState::MANUAL_VELOCITY;
      RCLCPP_INFO(get_logger(), "Altura de despegue alcanzada. Entrando en MANUAL_VELOCITY.");
    }
  }

  void loop()
  {
    if (manual_cmd_timeout() && state_ == ControlState::MANUAL_VELOCITY) {
      latest_cmd_ = geometry_msgs::msg::Twist {};
    }

    publish_current_setpoint();
    update_offboard_request();
    update_auto_arm();
    update_takeoff_transition();

    if (state_ == ControlState::LANDING && has_extended_state_ && landed_) {
      state_ = ControlState::IDLE;
      RCLCPP_INFO(get_logger(), "Aterrizaje detectado. Estado IDLE.");
    }

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
      "state=%s connected=%s mode=%s armed=%s local=%s z=%.2f target_z=%.2f cmd=[%.2f %.2f %.2f %.2f] smooth=[%.2f %.2f %.2f %.2f]",
      state_name(),
      is_connected() ? "true" : "false",
      has_mavros_state_ ? mavros_state_.mode.c_str() : "?",
      is_armed() ? "true" : "false",
      has_local_position_ ? "true" : "false",
      current_z_,
      takeoff_target_z_,
      latest_cmd_.linear.x,
      latest_cmd_.linear.y,
      latest_cmd_.linear.z,
      latest_cmd_.angular.z,
      smoothed_cmd_.linear.x,
      smoothed_cmd_.linear.y,
      smoothed_cmd_.linear.z,
      smoothed_cmd_.angular.z);
  }

  void print_state()
  {
    RCLCPP_INFO(
      get_logger(),
      "MAVROS state=%s connected=%s mode=%s armed=%s landed=%s safety=%s pos=[%.2f %.2f %.2f] yaw=%.2f",
      state_name(),
      is_connected() ? "true" : "false",
      has_mavros_state_ ? mavros_state_.mode.c_str() : "?",
      is_armed() ? "true" : "false",
      landed_ ? "true" : "false",
      safety_ok() ? "true" : "false",
      current_x_,
      current_y_,
      current_z_,
      current_yaw_);
  }

  double loop_hz_{30.0};
  double takeoff_altitude_m_{2.0};
  double takeoff_acceptance_m_{0.25};

  bool require_safety_enable_{true};
  bool auto_arm_on_takeoff_{false};

  double max_xy_speed_{0.8};
  double max_z_speed_{0.4};
  double max_yaw_rate_{0.6};

  double max_xy_accel_{0.6};
  double max_z_accel_{0.35};
  double max_yaw_accel_{1.0};

  double max_xy_decel_{1.2};
  double max_z_decel_{0.8};
  double max_yaw_decel_{1.5};

  double manual_deadman_timeout_s_{0.30};
  int offboard_warmup_cycles_{30};
  int auto_arm_delay_cycles_{10};

  ControlState state_{ControlState::IDLE};

  bool safety_enabled_{false};
  std::string safety_reason_{"sin datos de safety todavia"};

  bool offboard_command_sent_{false};
  bool auto_arm_after_offboard_{false};
  int offboard_warmup_counter_{0};
  int auto_arm_delay_counter_{0};

  bool has_mavros_state_{false};
  bool has_local_position_{false};
  bool has_extended_state_{false};
  bool landed_{true};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_yaw_{0.0};
  double takeoff_target_z_{2.0};

  geometry_msgs::msg::Twist latest_cmd_ {};
  geometry_msgs::msg::Twist smoothed_cmd_ {};
  rclcpp::Time last_manual_cmd_time_;

  mavros_msgs::msg::State mavros_state_ {};

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_reason_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}
