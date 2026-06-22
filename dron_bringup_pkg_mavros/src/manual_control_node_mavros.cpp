/**
 * manual_control_node_mavros.cpp
 *
 * Nodo central de control del dron. Intermedia entre keyboard_node / mando /
 * nodo autónomo y MAVROS.  Soporta dos backends intercambiables en vuelo:
 *
 *   manual_mavlink
 *     Publica MANUAL_CONTROL MAVLink en /mavros/manual_control/send.
 *     PX4 debe estar en STABILIZED o ALTCTL.
 *     No requiere GPS ni VIO. El piloto tiene control total de actitud.
 *
 *   offboard_velocity
 *     Publica setpoints de velocidad local en /mavros/setpoint_raw/local.
 *     PX4 en OFFBOARD. Requiere estimación de posición válida (GPS o VIO).
 *     Permite comportamiento autónomo completo.
 *
 * Mapa de estados internos:
 *
 *   IDLE
 *     └─ 'b'/'g' ──► MANUAL_MAVLINK
 *     └─ 'o'     ──► PRE_OFFBOARD → OFFBOARD_VELOCITY
 *
 *   MANUAL_MAVLINK
 *     └─ 'o' ──► PRE_OFFBOARD
 *     └─ 'l' ──► LANDING
 *     └─ 'n' ──► IDLE
 *
 *   PRE_OFFBOARD (calienta setpoints, luego pide modo OFFBOARD)
 *     └─ modo OFFBOARD confirmado ──► OFFBOARD_VELOCITY
 *     └─ 'b'/'g' ──► MANUAL_MAVLINK
 *
 *   OFFBOARD_VELOCITY
 *     └─ 't' ──► TAKEOFF_VELOCITY
 *     └─ 'l' ──► LANDING
 *     └─ 'b'/'g' ──► MANUAL_MAVLINK
 *     └─ 'n' ──► IDLE
 *
 *   TAKEOFF_VELOCITY (sube verticalmente hasta takeoff_altitude_m)
 *     └─ altura alcanzada ──► OFFBOARD_VELOCITY
 *     └─ 'b'/'g' ──► MANUAL_MAVLINK
 *
 *   LANDING
 *     └─ landed detectado ──► IDLE / MANUAL_MAVLINK
 *
 * Topics suscritos:
 *   /manual/cmd_vel          geometry_msgs/Twist   velocidades del teclado/mando
 *   /manual/key              std_msgs/String        teclas de comando
 *   /manual/safety_enable    std_msgs/Bool          permiso de safety (opcional)
 *   /manual/safety_reason    std_msgs/String        motivo de bloqueo
 *   /mavros/state            mavros_msgs/State
 *   /mavros/local_position/odom  nav_msgs/Odometry
 *   /mavros/extended_state   mavros_msgs/ExtendedState
 *
 * Topics publicados:
 *   /mavros/manual_control/send   mavros_msgs/ManualControl  (backend manual)
 *   /mavros/setpoint_raw/local    mavros_msgs/PositionTarget  (backend offboard)
 *
 * Servicios usados:
 *   /mavros/set_mode
 *   /mavros/cmd/arming
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
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

using namespace std::chrono_literals;

// ════════════════════════════════════════════════════════════════════════════════

class ManualControlNode : public rclcpp::Node
{
public:
  ManualControlNode()
  : Node("manual_control_node_mavros")
  {
    // ── Parámetros ─────────────────────────────────────────────────────────────

    loop_hz_ = declare_parameter<double>("loop_hz", 30.0);

    // Backend de control
    control_backend_ = declare_parameter<std::string>("control_backend", "manual_mavlink");
    manual_mode_     = declare_parameter<std::string>("manual_mode", "STABILIZED");
    auto_set_manual_mode_on_start_ = declare_parameter<bool>("auto_set_manual_mode_on_start", true);

    // Despegue
    takeoff_altitude_m_  = declare_parameter<double>("takeoff_altitude_m", 2.0);
    takeoff_acceptance_m_ = declare_parameter<double>("takeoff_acceptance_m", 0.25);

    // Safety
    require_safety_enable_ = declare_parameter<bool>("require_safety_enable", false);
    auto_arm_on_takeoff_   = declare_parameter<bool>("auto_arm_on_takeoff", false);

    // Límites de velocidad
    max_xy_speed_  = declare_parameter<double>("max_xy_speed", 0.8);
    max_z_speed_   = declare_parameter<double>("max_z_speed",  0.4);
    max_yaw_rate_  = declare_parameter<double>("max_yaw_rate", 0.6);

    // Suavizado (slew limiter)
    max_xy_accel_  = declare_parameter<double>("max_xy_accel",  0.6);
    max_z_accel_   = declare_parameter<double>("max_z_accel",   0.35);
    max_yaw_accel_ = declare_parameter<double>("max_yaw_accel", 1.0);
    max_xy_decel_  = declare_parameter<double>("max_xy_decel",  1.2);
    max_z_decel_   = declare_parameter<double>("max_z_decel",   0.8);
    max_yaw_decel_ = declare_parameter<double>("max_yaw_decel", 1.5);

    // Temporización
    manual_deadman_timeout_s_ = declare_parameter<double>("manual_deadman_timeout_s", 0.30);
    offboard_warmup_cycles_   = declare_parameter<int>("offboard_warmup_cycles", 30);
    auto_arm_delay_cycles_    = declare_parameter<int>("auto_arm_delay_cycles",  10);

    // Backend manual_mavlink – escala de ejes MAVLink
    manual_throttle_initial_ = declare_parameter<double>("manual_throttle_initial", 0.0);
    manual_throttle_rate_    = declare_parameter<double>("manual_throttle_rate",    0.35);
    manual_throttle_min_     = declare_parameter<double>("manual_throttle_min",     0.0);
    manual_throttle_max_     = declare_parameter<double>("manual_throttle_max",     1.0);
    manual_axis_scale_       = declare_parameter<double>("manual_axis_scale",       1000.0);

    manual_throttle_ = clamp_throttle(manual_throttle_initial_);

    // ── Publicadores ────────────────────────────────────────────────────────────

    // Offboard: setpoints de velocidad local (FRAME_BODY_NED)
    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);

    // Manual: joystick MAVLink
    manual_control_pub_ = create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);

    // ── Suscriptores ────────────────────────────────────────────────────────────

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

    // SensorDataQoS es el perfil correcto para odometría (best-effort, volatile)
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&ManualControlNode::odom_cb, this, std::placeholders::_1));

    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", 10,
      std::bind(&ManualControlNode::extended_state_cb, this, std::placeholders::_1));

    // ── Clientes de servicio ─────────────────────────────────────────────────────

    arming_client_   = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // ── Timer principal ──────────────────────────────────────────────────────────

    last_manual_cmd_time_ = now();

    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ManualControlNode::loop, this));

    // Estado inicial coherente con backend
    state_ = is_manual_backend() ? ControlState::MANUAL_MAVLINK : ControlState::IDLE;

    RCLCPP_INFO(get_logger(), "manual_control_node_mavros iniciado");
    RCLCPP_INFO(get_logger(), "  Backend : %s",  control_backend_.c_str());
    RCLCPP_INFO(get_logger(), "  Modo PX4: %s",  manual_mode_.c_str());
    RCLCPP_INFO(get_logger(), "  Estado  : %s",  state_name());
    RCLCPP_INFO(get_logger(),
      "  b/g = modo teclado MAVLINK  |  o = modo OFFBOARD autónomo");
  }

private:
  // ════════════════════════════════════════════════════════════════════════════
  // Tipos / enums
  // ════════════════════════════════════════════════════════════════════════════

  enum class ControlState {
    IDLE,
    MANUAL_MAVLINK,     ///< Teclado como joystick MAVLink → PX4 STABILIZED/ALTCTL
    PRE_OFFBOARD,       ///< Calentando setpoints antes de pedir OFFBOARD
    TAKEOFF_VELOCITY,   ///< Subida vertical en Offboard
    OFFBOARD_VELOCITY,  ///< Vuelo manual/autónomo en Offboard
    LANDING             ///< AUTO.LAND activo
  };

  const char * state_name() const
  {
    switch (state_) {
      case ControlState::IDLE:              return "IDLE";
      case ControlState::MANUAL_MAVLINK:    return "MANUAL_MAVLINK";
      case ControlState::PRE_OFFBOARD:      return "PRE_OFFBOARD";
      case ControlState::TAKEOFF_VELOCITY:  return "TAKEOFF_VELOCITY";
      case ControlState::OFFBOARD_VELOCITY: return "OFFBOARD_VELOCITY";
      case ControlState::LANDING:           return "LANDING";
      default:                              return "UNKNOWN";
    }
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Utilidades matemáticas
  // ════════════════════════════════════════════════════════════════════════════

  static double clamp_sym(double v, double limit)
  {
    return v >  limit ?  limit :
           v < -limit ? -limit : v;
  }

  static double clamp_range(double v, double lo, double hi)
  {
    return v < lo ? lo : v > hi ? hi : v;
  }

  double clamp_throttle(double v) const
  {
    return clamp_range(v, manual_throttle_min_, manual_throttle_max_);
  }

  /// Limita la tasa de cambio de `current` hacia `target`.
  static double slew_limit(double current, double target, double max_rate, double dt)
  {
    const double max_delta = max_rate * dt;
    const double delta     = target - current;
    if (delta >  max_delta) { return current + max_delta; }
    if (delta < -max_delta) { return current - max_delta; }
    return target;
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Predicados de estado
  // ════════════════════════════════════════════════════════════════════════════

  bool is_connected()  const { return has_mavros_state_ && mavros_state_.connected; }
  bool is_armed()      const { return has_mavros_state_ && mavros_state_.armed; }
  bool is_offboard()   const { return has_mavros_state_ && mavros_state_.mode == "OFFBOARD"; }
  bool is_manual_backend()   const { return control_backend_ == "manual_mavlink"; }
  bool is_offboard_backend() const { return control_backend_ == "offboard_velocity"; }

  bool safety_ok() const
  {
    if (!require_safety_enable_) { return true; }
    return safety_enabled_;
  }

  bool manual_cmd_timed_out() const
  {
    return (now() - last_manual_cmd_time_).seconds() > manual_deadman_timeout_s_;
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Callbacks de suscriptores
  // ════════════════════════════════════════════════════════════════════════════

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Clamping inmediato a límites físicos
    latest_cmd_.linear.x  = clamp_sym(msg->linear.x,  max_xy_speed_);
    latest_cmd_.linear.y  = clamp_sym(msg->linear.y,  max_xy_speed_);
    latest_cmd_.linear.z  = clamp_sym(msg->linear.z,  max_z_speed_);
    latest_cmd_.angular.z = clamp_sym(msg->angular.z, max_yaw_rate_);
    last_manual_cmd_time_ = now();
  }

  void key_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) { return; }
    const char key = msg->data[0];

    switch (key) {
      case 'b': case 'g': enable_manual_mavlink_mode(); break;
      case 'o':           prepare_offboard();           break;
      case 'm':           arm();                        break;
      case 'n':           disarm();                     break;
      case 't':           start_takeoff();              break;
      case 'l':           land();                       break;
      case 'x':
        manual_throttle_ = 0.0;
        stop_motion();
        RCLCPP_WARN(get_logger(), "Throttle a cero (tecla x)");
        break;
      case ' ': case 'z':
        stop_motion();
        break;
      case 'p':
        print_state();
        break;
      default:
        break;
    }
  }

  void safety_cb(const std_msgs::msg::Bool::SharedPtr msg)   { safety_enabled_ = msg->data; }
  void safety_reason_cb(const std_msgs::msg::String::SharedPtr msg) { safety_reason_ = msg->data; }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    mavros_state_     = *msg;
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
    double roll = 0.0, pitch = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
  }

  void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    landed_             = msg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
    has_extended_state_ = true;
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Llamadas a servicios MAVROS
  // ════════════════════════════════════════════════════════════════════════════

  void call_set_mode(const std::string & mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Servicio /mavros/set_mode no disponible todavía");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode   = 0;
    req->custom_mode = mode;
    set_mode_client_->async_send_request(
      req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
        RCLCPP_INFO(get_logger(), "SET_MODE '%s' → mode_sent=%s",
          mode.c_str(), fut.get()->mode_sent ? "true" : "false");
      });
  }

  void call_arm(bool arm_value)
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Servicio /mavros/cmd/arming no disponible todavía");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm_value;
    arming_client_->async_send_request(
      req,
      [this, arm_value](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
        const auto res = fut.get();
        RCLCPP_INFO(get_logger(), "%s → success=%s result=%u",
          arm_value ? "ARM" : "DISARM",
          res->success ? "true" : "false",
          res->result);
      });
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Transiciones de estado
  // ════════════════════════════════════════════════════════════════════════════

  /// Activar backend de teclado MAVLink (STABILIZED / ALTCTL).
  void enable_manual_mavlink_mode()
  {
    control_backend_            = "manual_mavlink";
    state_                      = ControlState::MANUAL_MAVLINK;
    offboard_command_sent_      = false;
    auto_arm_after_offboard_    = false;
    manual_mode_request_sent_   = false;
    stop_motion();

    if (is_connected()) {
      call_set_mode(manual_mode_);
      manual_mode_request_sent_ = true;
    }
    RCLCPP_WARN(get_logger(),
      "→ MANUAL_MAVLINK. PX4 debe estar en %s. COM_RC_IN_MODE debe permitir MAVLink joystick.",
      manual_mode_.c_str());
  }

  /// Activar backend Offboard (requiere estimación de posición válida).
  void prepare_offboard()
  {
    if (!is_connected()) {
      RCLCPP_WARN(get_logger(), "No puedo activar OFFBOARD: MAVROS no está conectado a la FCU");
      return;
    }
    control_backend_          = "offboard_velocity";
    state_                    = ControlState::PRE_OFFBOARD;
    offboard_warmup_counter_  = 0;
    offboard_command_sent_    = false;
    manual_mode_request_sent_ = false;
    stop_motion();
    RCLCPP_INFO(get_logger(),
      "→ PRE_OFFBOARD. Publicando setpoints de calentamiento (%d ciclos)…",
      offboard_warmup_cycles_);
  }

  void arm()
  {
    if (!safety_ok()) {
      RCLCPP_WARN(get_logger(), "ARM bloqueado por safety: %s", safety_reason_.c_str());
      return;
    }
    // En modo manual, asegurar throttle bajo antes de armar
    if (is_manual_backend()) {
      manual_throttle_ = 0.0;
    }
    call_arm(true);
  }

  void disarm()
  {
    call_arm(false);
    state_                   = ControlState::IDLE;
    manual_throttle_         = 0.0;
    auto_arm_after_offboard_ = false;
    offboard_command_sent_   = false;
    stop_motion();
    RCLCPP_WARN(get_logger(), "DISARM solicitado");
  }

  void start_takeoff()
  {
    if (is_manual_backend()) {
      RCLCPP_WARN(get_logger(),
        "Despegue automático no disponible en backend manual_mavlink. "
        "Usa 'o' para activar Offboard, luego 't'.");
      return;
    }
    if (!safety_ok()) {
      RCLCPP_WARN(get_logger(), "TAKEOFF bloqueado por safety: %s", safety_reason_.c_str());
      return;
    }
    if (!has_local_position_) {
      RCLCPP_WARN(get_logger(), "TAKEOFF bloqueado: sin odometría válida (/mavros/local_position/odom)");
      return;
    }
    takeoff_target_z_ = current_z_ + takeoff_altitude_m_;
    state_            = ControlState::TAKEOFF_VELOCITY;
    stop_motion();

    // Si no estamos ya en Offboard, iniciar secuencia de calentamiento
    if (!is_offboard()) {
      offboard_warmup_counter_ = 0;
      offboard_command_sent_   = false;
    }
    if (auto_arm_on_takeoff_ && !is_armed()) {
      auto_arm_after_offboard_  = true;
      auto_arm_delay_counter_   = 0;
    }
    RCLCPP_INFO(get_logger(),
      "→ TAKEOFF_VELOCITY: z_actual=%.2f m  objetivo=%.2f m",
      current_z_, takeoff_target_z_);
  }

  void land()
  {
    state_           = ControlState::LANDING;
    manual_throttle_ = 0.0;
    stop_motion();
    call_set_mode("AUTO.LAND");
    RCLCPP_WARN(get_logger(), "→ LANDING (AUTO.LAND)");
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Lógica de movimiento y publicación
  // ════════════════════════════════════════════════════════════════════════════

  void stop_motion()
  {
    latest_cmd_           = geometry_msgs::msg::Twist{};
    smoothed_cmd_         = geometry_msgs::msg::Twist{};
    last_manual_cmd_time_ = now();
  }

  /// Aplica slew limiter a latest_cmd → smoothed_cmd.
  void update_smoothed_cmd()
  {
    const double dt = 1.0 / loop_hz_;

    // Tasa de aceleración/deceleración diferenciada
    const double rate_x = (std::fabs(latest_cmd_.linear.x) < std::fabs(smoothed_cmd_.linear.x))
                          ? max_xy_decel_ : max_xy_accel_;
    const double rate_y = (std::fabs(latest_cmd_.linear.y) < std::fabs(smoothed_cmd_.linear.y))
                          ? max_xy_decel_ : max_xy_accel_;
    const double rate_z = (std::fabs(latest_cmd_.linear.z) < std::fabs(smoothed_cmd_.linear.z))
                          ? max_z_decel_  : max_z_accel_;
    const double rate_w = (std::fabs(latest_cmd_.angular.z) < std::fabs(smoothed_cmd_.angular.z))
                          ? max_yaw_decel_ : max_yaw_accel_;

    smoothed_cmd_.linear.x  = slew_limit(smoothed_cmd_.linear.x,  latest_cmd_.linear.x,  rate_x, dt);
    smoothed_cmd_.linear.y  = slew_limit(smoothed_cmd_.linear.y,  latest_cmd_.linear.y,  rate_y, dt);
    smoothed_cmd_.linear.z  = slew_limit(smoothed_cmd_.linear.z,  latest_cmd_.linear.z,  rate_z, dt);
    smoothed_cmd_.angular.z = slew_limit(smoothed_cmd_.angular.z, latest_cmd_.angular.z, rate_w, dt);
  }

  /// Integra el eje vertical suavizado en el throttle manual [0,1].
  void update_manual_throttle()
  {
    if (std::fabs(smoothed_cmd_.linear.z) < 1e-4) { return; }

    const double dt         = 1.0 / loop_hz_;
    const double normalized = clamp_sym(smoothed_cmd_.linear.z / std::max(1e-6, max_z_speed_), 1.0);
    manual_throttle_        = clamp_throttle(manual_throttle_ + normalized * manual_throttle_rate_ * dt);
  }

  // ── Backend manual MAVLink ──────────────────────────────────────────────────

  void maybe_request_manual_mode_on_start()
  {
    if (!auto_set_manual_mode_on_start_) { return; }
    if (!is_manual_backend())            { return; }
    if (!is_connected())                 { return; }
    if (manual_mode_request_sent_)       { return; }

    // Ya está en el modo correcto, solo marcamos
    if (has_mavros_state_ && mavros_state_.mode == manual_mode_) {
      manual_mode_request_sent_ = true;
      return;
    }
    call_set_mode(manual_mode_);
    manual_mode_request_sent_ = true;
  }

  void publish_manual_control()
  {
    update_smoothed_cmd();
    update_manual_throttle();

    mavros_msgs::msg::ManualControl msg {};
    msg.header.stamp     = now();
    msg.header.frame_id  = "keyboard_mavlink";

    // MANUAL_CONTROL MAVLink:
    //   x  = pitch   positivo = nose-down (adelante)
    //   y  = roll    positivo = derecha
    //   z  = throttle [0…1000]
    //   r  = yaw     positivo = derecha
    //
    // Convención teclado/Twist:
    //   linear.x  = adelante (+), atrás (−)
    //   linear.y  = izquierda (+), derecha (−)
    //   angular.z = yaw-left (+), yaw-right (−)

    const double norm_xy  = std::max(1e-6, max_xy_speed_);
    const double norm_yaw = std::max(1e-6, max_yaw_rate_);

    msg.x = static_cast<float>(clamp_sym(smoothed_cmd_.linear.x  / norm_xy,  1.0) * manual_axis_scale_);
    msg.y = static_cast<float>(clamp_sym(-smoothed_cmd_.linear.y / norm_xy,  1.0) * manual_axis_scale_);
    msg.z = static_cast<float>(manual_throttle_ * manual_axis_scale_);
    msg.r = static_cast<float>(clamp_sym(-smoothed_cmd_.angular.z / norm_yaw, 1.0) * manual_axis_scale_);
    msg.buttons = 0;

    manual_control_pub_->publish(msg);
  }

  // ── Backend Offboard velocity ───────────────────────────────────────────────

  /**
   * Publica un setpoint de velocidad en FRAME_BODY_NED.
   * Convención NED:  x=adelante, y=derecha, z=abajo.
   * Convención ROS:  x=adelante, y=izquierda, z=arriba.
   */
  void publish_body_velocity_setpoint(
    double forward, double left, double up, double yaw_rate_left)
  {
    mavros_msgs::msg::PositionTarget sp {};
    sp.header.stamp      = now();
    sp.header.frame_id   = "base_link";
    sp.coordinate_frame  = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;

    sp.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_PX  |
      mavros_msgs::msg::PositionTarget::IGNORE_PY  |
      mavros_msgs::msg::PositionTarget::IGNORE_PZ  |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW;

    // ROS → NED
    sp.velocity.x = forward;
    sp.velocity.y = -left;
    sp.velocity.z = -up;
    sp.yaw_rate   = -yaw_rate_left;

    setpoint_pub_->publish(sp);
  }

  void publish_offboard_setpoint()
  {
    switch (state_) {
      case ControlState::TAKEOFF_VELOCITY: {
        const double remaining = takeoff_target_z_ - current_z_;
        const double up = (remaining > takeoff_acceptance_m_) ? max_z_speed_ : 0.0;
        publish_body_velocity_setpoint(0.0, 0.0, up, 0.0);
        break;
      }
      case ControlState::OFFBOARD_VELOCITY:
        update_smoothed_cmd();
        publish_body_velocity_setpoint(
          smoothed_cmd_.linear.x,
          smoothed_cmd_.linear.y,
          smoothed_cmd_.linear.z,
          smoothed_cmd_.angular.z);
        break;
      default:
        // PRE_OFFBOARD y cualquier otro: setpoint cero de calentamiento
        publish_body_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
        break;
    }
  }

  /// Gestiona la transición PRE_OFFBOARD → OFFBOARD tras el calentamiento.
  void update_offboard_request()
  {
    if (state_ != ControlState::PRE_OFFBOARD &&
        state_ != ControlState::TAKEOFF_VELOCITY)
    {
      return;
    }

    // Ya en OFFBOARD: avanzar estado si venimos de PRE_OFFBOARD
    if (is_offboard()) {
      if (state_ == ControlState::PRE_OFFBOARD) {
        state_ = ControlState::OFFBOARD_VELOCITY;
        RCLCPP_INFO(get_logger(), "PX4 confirmó OFFBOARD → OFFBOARD_VELOCITY");
      }
      return;
    }

    // Esperar si ya enviamos la petición
    if (offboard_command_sent_) { return; }

    // Ciclos de calentamiento
    if (offboard_warmup_counter_ < offboard_warmup_cycles_) {
      ++offboard_warmup_counter_;
      return;
    }

    call_set_mode("OFFBOARD");
    offboard_command_sent_ = true;
  }

  /// Auto-armado cuando se activa OFFBOARD (si auto_arm_on_takeoff=true).
  void update_auto_arm()
  {
    if (!auto_arm_after_offboard_) { return; }
    if (!is_offboard())            { return; }

    if (auto_arm_delay_counter_ < auto_arm_delay_cycles_) {
      ++auto_arm_delay_counter_;
      return;
    }
    auto_arm_after_offboard_ = false;
    if (!is_armed()) { arm(); }
  }

  /// Detecta que se alcanzó la altura objetivo en despegue.
  void update_takeoff_transition()
  {
    if (state_ != ControlState::TAKEOFF_VELOCITY || !has_local_position_) { return; }

    if ((takeoff_target_z_ - current_z_) <= takeoff_acceptance_m_) {
      stop_motion();
      state_ = ControlState::OFFBOARD_VELOCITY;
      RCLCPP_INFO(get_logger(),
        "Altura %.2f m alcanzada → OFFBOARD_VELOCITY", takeoff_target_z_);
    }
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Loop principal
  // ════════════════════════════════════════════════════════════════════════════

  void loop()
  {
    // Deadman: si no llegan comandos, forzar cero (excepto en estados autónomos)
    if (manual_cmd_timed_out() &&
        (state_ == ControlState::MANUAL_MAVLINK ||
         state_ == ControlState::OFFBOARD_VELOCITY))
    {
      latest_cmd_.linear.x  = 0.0;
      latest_cmd_.linear.y  = 0.0;
      latest_cmd_.linear.z  = 0.0;
      latest_cmd_.angular.z = 0.0;
    }

    // ── Despacho por backend ────────────────────────────────────────────────

    if (is_manual_backend()) {
      state_ = ControlState::MANUAL_MAVLINK;
      maybe_request_manual_mode_on_start();
      publish_manual_control();

    } else if (is_offboard_backend()) {
      publish_offboard_setpoint();
      update_offboard_request();
      update_auto_arm();
      update_takeoff_transition();

    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "control_backend desconocido: '%s'. Revertiendo a manual_mavlink.",
        control_backend_.c_str());
      control_backend_ = "manual_mavlink";
    }

    // ── Transición post-aterrizaje ──────────────────────────────────────────

    if (state_ == ControlState::LANDING && has_extended_state_ && landed_) {
      state_ = is_manual_backend() ? ControlState::MANUAL_MAVLINK : ControlState::IDLE;
      RCLCPP_INFO(get_logger(), "Aterrizaje confirmado por extended_state → %s", state_name());
    }

    // ── Avisos de safety ────────────────────────────────────────────────────

    if (require_safety_enable_ && !safety_enabled_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Safety NO OK: %s", safety_reason_.c_str());
    }

    // ── Telemetría de diagnóstico (cada 5 s) ────────────────────────────────

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "[%s/%s] connected=%s px4_mode=%s armed=%s landed=%s "
      "local=%s z=%.2f thr=%.2f  cmd=[%.2f %.2f %.2f %.2f]",
      control_backend_.c_str(), state_name(),
      is_connected() ? "Y" : "N",
      has_mavros_state_ ? mavros_state_.mode.c_str() : "?",
      is_armed()    ? "Y" : "N",
      landed_       ? "Y" : "N",
      has_local_position_ ? "Y" : "N",
      current_z_, manual_throttle_,
      latest_cmd_.linear.x, latest_cmd_.linear.y,
      latest_cmd_.linear.z, latest_cmd_.angular.z);
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Debug
  // ════════════════════════════════════════════════════════════════════════════

  void print_state()
  {
    RCLCPP_INFO(get_logger(),
      "=== Estado manual_control_node ===\n"
      "  backend   : %s\n"
      "  estado    : %s\n"
      "  connected : %s\n"
      "  px4_mode  : %s\n"
      "  armed     : %s\n"
      "  landed    : %s\n"
      "  safety    : %s\n"
      "  pos       : [%.3f  %.3f  %.3f] m\n"
      "  yaw       : %.3f rad\n"
      "  throttle  : %.3f\n"
      "  cmd_vel   : [%.3f  %.3f  %.3f  %.3f]",
      control_backend_.c_str(), state_name(),
      is_connected() ? "true" : "false",
      has_mavros_state_ ? mavros_state_.mode.c_str() : "?",
      is_armed()   ? "true" : "false",
      landed_      ? "true" : "false",
      safety_ok()  ? "OK"   : safety_reason_.c_str(),
      current_x_, current_y_, current_z_,
      current_yaw_, manual_throttle_,
      latest_cmd_.linear.x, latest_cmd_.linear.y,
      latest_cmd_.linear.z, latest_cmd_.angular.z);
  }

  // ════════════════════════════════════════════════════════════════════════════
  // Miembros – parámetros
  // ════════════════════════════════════════════════════════════════════════════

  double      loop_hz_{30.0};
  std::string control_backend_{"manual_mavlink"};
  std::string manual_mode_{"STABILIZED"};
  bool        auto_set_manual_mode_on_start_{true};
  bool        manual_mode_request_sent_{false};

  double takeoff_altitude_m_{2.0};
  double takeoff_acceptance_m_{0.25};

  bool   require_safety_enable_{false};
  bool   auto_arm_on_takeoff_{false};

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
  int    offboard_warmup_cycles_{30};
  int    auto_arm_delay_cycles_{10};

  double manual_throttle_initial_{0.0};
  double manual_throttle_rate_{0.35};
  double manual_throttle_min_{0.0};
  double manual_throttle_max_{1.0};
  double manual_axis_scale_{1000.0};

  // ════════════════════════════════════════════════════════════════════════════
  // Miembros – estado en tiempo de ejecución
  // ════════════════════════════════════════════════════════════════════════════

  ControlState state_{ControlState::MANUAL_MAVLINK};

  bool        safety_enabled_{false};
  std::string safety_reason_{"sin datos de safety todavía"};

  bool offboard_command_sent_{false};
  bool auto_arm_after_offboard_{false};
  int  offboard_warmup_counter_{0};
  int  auto_arm_delay_counter_{0};

  bool   has_mavros_state_{false};
  bool   has_local_position_{false};
  bool   has_extended_state_{false};
  bool   landed_{true};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_yaw_{0.0};
  double takeoff_target_z_{2.0};
  double manual_throttle_{0.0};

  geometry_msgs::msg::Twist latest_cmd_{};
  geometry_msgs::msg::Twist smoothed_cmd_{};
  rclcpp::Time              last_manual_cmd_time_;

  mavros_msgs::msg::State mavros_state_{};

  // ════════════════════════════════════════════════════════════════════════════
  // Miembros – ROS
  // ════════════════════════════════════════════════════════════════════════════

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr  manual_control_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            key_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            safety_reason_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr          state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr  extended_state_sub_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr     set_mode_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

// ════════════════════════════════════════════════════════════════════════════════

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}
