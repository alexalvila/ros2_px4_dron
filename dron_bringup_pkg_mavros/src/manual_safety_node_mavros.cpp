/**
 * manual_safety_node_mavros.cpp
 *
 * Nodo watchdog que evalúa condiciones de seguridad y publica dos topics:
 *
 *   /manual/safety_enable  (std_msgs/Bool)    true = operación permitida
 *   /manual/safety_reason  (std_msgs/String)  motivo de bloqueo (o "OK")
 *
 * manual_control_node_mavros consulta estos topics antes de armar o despegar.
 *
 * Condiciones evaluadas:
 *   1. Conexión MAVROS → FCU   (require_mavros_connected, defecto: true)
 *   2. Batería mínima          (min_battery_remaining, defecto: 0.20 → 20 %)
 *   3. Vehículo en tierra      (block_if_landed, defecto: false)
 *
 * Parámetros ROS 2:
 *   publish_hz              – frecuencia de publicación (Hz, defecto 10)
 *   min_battery_remaining   – fracción mínima de batería [0-1] (defecto 0.20)
 *   block_if_landed         – bloquear si el dron está en tierra (defecto false)
 *   require_mavros_connected – requerir conexión FCU (defecto true)
 */

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class ManualSafetyNode : public rclcpp::Node
{
public:
  ManualSafetyNode()
  : Node("manual_safety_node_mavros")
  {
    publish_hz_              = declare_parameter<double>("publish_hz",              10.0);
    min_battery_remaining_   = declare_parameter<double>("min_battery_remaining",   0.20);
    block_if_landed_         = declare_parameter<bool>  ("block_if_landed",         false);
    require_mavros_connected_ = declare_parameter<bool> ("require_mavros_connected", true);

    enable_pub_ = create_publisher<std_msgs::msg::Bool>  ("/manual/safety_enable", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/manual/safety_reason", 10);

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&ManualSafetyNode::state_cb, this, std::placeholders::_1));

    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", 10,
      std::bind(&ManualSafetyNode::extended_state_cb, this, std::placeholders::_1));

    // SensorDataQoS porque /mavros/battery puede publicar con best-effort
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::SensorDataQoS(),
      std::bind(&ManualSafetyNode::battery_cb, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ManualSafetyNode::loop, this));

    RCLCPP_INFO(get_logger(), "manual_safety_node_mavros listo");
    RCLCPP_INFO(get_logger(), "  require_mavros_connected = %s",
      require_mavros_connected_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  min_battery_remaining    = %.0f %%",
      min_battery_remaining_ * 100.0);
    RCLCPP_INFO(get_logger(), "  block_if_landed          = %s",
      block_if_landed_ ? "true" : "false");
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    state_     = *msg;
    has_state_ = true;
  }

  void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    extended_state_     = *msg;
    has_extended_state_ = true;
  }

  void battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    battery_     = *msg;
    has_battery_ = true;
  }

  /// Añade un motivo a la cadena de razones de bloqueo.
  static void add_reason(std::ostringstream & oss, bool & first, const std::string & reason)
  {
    if (!first) { oss << "; "; }
    oss << reason;
    first = false;
  }

  void loop()
  {
    bool              enabled      = true;
    bool              first_reason = true;
    std::ostringstream reasons;

    // ── Condición 1: conexión MAVROS ────────────────────────────────────────

    if (require_mavros_connected_) {
      if (!has_state_) {
        enabled = false;
        add_reason(reasons, first_reason, "sin /mavros/state");
      } else if (!state_.connected) {
        enabled = false;
        add_reason(reasons, first_reason, "MAVROS no conectado a la FCU");
      }
    }

    // ── Condición 2: batería ────────────────────────────────────────────────

    if (!has_battery_) {
      // Aviso informativo, no bloqueante (puede no haber sensor de batería)
      add_reason(reasons, first_reason, "sin /mavros/battery");
    } else if (battery_.percentage >= 0.0f &&
               battery_.percentage < static_cast<float>(min_battery_remaining_))
    {
      enabled = false;
      std::ostringstream r;
      r << "batería baja: " << static_cast<int>(battery_.percentage * 100.0f) << "%";
      add_reason(reasons, first_reason, r.str());
    }

    // ── Condición 3: en tierra ──────────────────────────────────────────────

    if (block_if_landed_) {
      if (!has_extended_state_) {
        add_reason(reasons, first_reason, "sin /mavros/extended_state");
      } else if (extended_state_.landed_state ==
                 mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)
      {
        enabled = false;
        add_reason(reasons, first_reason, "vehículo en tierra");
      }
    }

    // ── Publicar ────────────────────────────────────────────────────────────

    std_msgs::msg::Bool enable_msg;
    enable_msg.data = enabled;
    enable_pub_->publish(enable_msg);

    std_msgs::msg::String reason_msg;
    reason_msg.data = enabled ? "OK" : reasons.str();
    reason_pub_->publish(reason_msg);

    // Log periódico solo cuando hay algo que reportar
    if (!enabled) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Safety BLOQUEADO: %s", reasons.str().c_str());
    }
  }

  // ── Parámetros ───────────────────────────────────────────────────────────────

  double publish_hz_{10.0};
  double min_battery_remaining_{0.20};
  bool   block_if_landed_{false};
  bool   require_mavros_connected_{true};

  // ── Estado ───────────────────────────────────────────────────────────────────

  bool has_state_{false};
  bool has_extended_state_{false};
  bool has_battery_{false};

  mavros_msgs::msg::State         state_{};
  mavros_msgs::msg::ExtendedState extended_state_{};
  sensor_msgs::msg::BatteryState  battery_{};

  // ── ROS ──────────────────────────────────────────────────────────────────────

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr  battery_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
