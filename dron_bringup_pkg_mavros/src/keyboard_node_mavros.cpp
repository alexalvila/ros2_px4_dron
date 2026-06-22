/**
 * keyboard_node_mavros.cpp
 *
 * Lee el teclado y publica dos topics:
 *   /manual/cmd_vel  (geometry_msgs/Twist) – velocidades de movimiento continuo.
 *   /manual/key      (std_msgs/String)      – teclas de comando puntuales.
 *
 * El nodo NO toca MAVLink directamente: solo genera intenciones.
 * manual_control_node_mavros las traduce según el backend activo
 * (manual_mavlink / offboard_velocity).
 *
 * Mapa de teclas
 * ──────────────
 *  b / g  → modo teclado MAVLink  (PX4 STABILIZED o ALTCTL)
 *  o      → modo autónomo Offboard por velocidad local
 *  m      → armar
 *  n      → desarmar
 *  l      → aterrizar (AUTO.LAND)
 *  t      → despegar (solo válido en modo Offboard)
 *  x      → throttle a cero (emergencia suave)
 *
 *  w/s    → pitch  adelante / atrás
 *  a/d    → roll   izquierda / derecha
 *  r/f    → throttle/altitud  subir / bajar
 *  q/e    → yaw    izquierda / derecha
 *  SPACE/z→ nivelar ejes, mantiene throttle
 *
 *  +/-    → aumentar / disminuir sensibilidad global
 *  p      → imprimir estado del nodo de control
 *  h      → mostrar esta ayuda
 *  Ctrl+C → salir
 */

#include <algorithm>
#include <cstdio>
#include <string>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
const char * HELP = R"HELP(
Modo teclado MAVROS – dron fisico (Pixhawk 6C / PX4)
=====================================================
b / g  : modo teclado MAVLink → PX4 en STABILIZED
o      : modo autonomo Offboard (requiere GPS/VIO valido)
m      : armar
n      : desarmar
l      : aterrizar (AUTO.LAND)
t      : despegar automatico (solo en modo Offboard)
x      : throttle a cero (emergencia suave)

Movimiento (momentaneo mientras la tecla esta activa):
  w/s  : pitch adelante / atras
  a/d  : roll izquierda / derecha
  r/f  : subir / bajar
  q/e  : yaw izquierda / derecha
  SPACE / z : nivelar roll‑pitch‑yaw (mantiene throttle)

+/-  : aumentar / disminuir sensibilidad
p    : imprimir estado del nodo de control
h    : mostrar esta ayuda
Ctrl+C : salir

)HELP";
}  // namespace

class KeyboardNode : public rclcpp::Node
{
public:
  KeyboardNode()
  : Node("keyboard_node_mavros")
  {
    publish_hz_         = declare_parameter<double>("publish_hz", 30.0);
    linear_speed_       = declare_parameter<double>("linear_speed", 0.75);
    vertical_speed_     = declare_parameter<double>("vertical_speed", 0.35);
    key_timeout_s_      = declare_parameter<double>("key_timeout_s", 0.20);
    linear_incremental_ = declare_parameter<double>("linear_incremental", 1.0);
    vertical_incremental_ = declare_parameter<double>("vertical_incremental", 1.0);
    yaw_incremental_    = declare_parameter<double>("yaw_incremental", 1.0);

    // boost_* se declaran para no romper YAML existentes, pero no se usan
    // actualmente (el mecanismo de incremento es más flexible).
    declare_parameter<double>("boost_linear_speed", 1.25);
    declare_parameter<double>("boost_vertical_speed", 0.60);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/manual/cmd_vel", 10);
    key_pub_ = create_publisher<std_msgs::msg::String>("/manual/key", 10);

    last_key_time_ = now();

    // Poner stdin en modo raw no-bloqueante
    if (tcgetattr(STDIN_FILENO, &original_termios_) == 0) {
      termios_available_ = true;
      struct termios raw = original_termios_;
      raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
      raw.c_cc[VMIN]  = 0;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
      fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo configurar el terminal en modo raw. "
                                "El teclado puede no funcionar correctamente.");
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KeyboardNode::loop, this));

    RCLCPP_INFO(get_logger(), "%s", HELP);
  }

  ~KeyboardNode() override { restore_terminal(); }

private:
  // ── Terminal ────────────────────────────────────────────────────────────────

  void restore_terminal()
  {
    if (termios_available_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      termios_available_ = false;
    }
  }

  char read_key()
  {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    struct timeval timeout {};
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;

    if (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout) > 0 &&
        FD_ISSET(STDIN_FILENO, &set))
    {
      char c = 0;
      if (read(STDIN_FILENO, &c, 1) == 1) {
        return c;
      }
    }
    return 0;
  }

  // ── Publicación ─────────────────────────────────────────────────────────────

  void publish_key(char key)
  {
    std_msgs::msg::String msg;
    msg.data = std::string(1, key);
    key_pub_->publish(msg);
  }

  // ── Lógica de teclado ───────────────────────────────────────────────────────

  static bool is_motion_key(char key)
  {
    return key == 'w' || key == 's' || key == 'a' || key == 'd' ||
           key == 'r' || key == 'f' || key == 'q' || key == 'e' ||
           key == ' ' || key == 'z';
  }

  static bool is_command_key(char key)
  {
    return key == 'b' || key == 'g' || key == 'o' ||
           key == 'm' || key == 'n' ||
           key == 't' || key == 'l' ||
           key == 'x' || key == 'p' || key == 'h';
  }

  void handle_key(char key)
  {
    if (key == 0)  { return; }
    if (key == 3)  { RCLCPP_INFO(get_logger(), "Ctrl+C detectado. Cerrando."); rclcpp::shutdown(); return; }
    if (key == 'h') { RCLCPP_INFO(get_logger(), "%s", HELP); return; }

    // Ajuste de sensibilidad
    if (key == '+') {
      linear_incremental_   += INCREMENT_STEP;
      vertical_incremental_ += INCREMENT_STEP;
      yaw_incremental_      += INCREMENT_STEP;
      RCLCPP_INFO(get_logger(),
        "Sensibilidad aumentada → linear=%.2f  vertical=%.2f  yaw=%.2f",
        linear_incremental_, vertical_incremental_, yaw_incremental_);
      return;
    }
    if (key == '-') {
      linear_incremental_   = std::max(0.10, linear_incremental_   - INCREMENT_STEP);
      vertical_incremental_ = std::max(0.10, vertical_incremental_ - INCREMENT_STEP);
      yaw_incremental_      = std::max(0.01, yaw_incremental_      - INCREMENT_STEP);
      RCLCPP_INFO(get_logger(),
        "Sensibilidad reducida  → linear=%.2f  vertical=%.2f  yaw=%.2f",
        linear_incremental_, vertical_incremental_, yaw_incremental_);
      return;
    }

    // Teclas de movimiento: guardar para publicación continua hasta timeout
    if (is_motion_key(key)) {
      last_key_      = key;
      last_key_time_ = now();
    }

    // Publicar tecla (movimiento y comando) al nodo de control
    if (is_motion_key(key) || is_command_key(key)) {
      publish_key(key);
    }
  }

  bool key_is_recent() const
  {
    return (now() - last_key_time_).seconds() <= key_timeout_s_;
  }

  geometry_msgs::msg::Twist build_cmd_from_last_key() const
  {
    geometry_msgs::msg::Twist cmd {};
    if (!key_is_recent()) { return cmd; }

    const double linear   = linear_speed_   * linear_incremental_;
    const double vertical = vertical_speed_ * vertical_incremental_;
    const double yaw_v    = YAW_STEP_BASE   * yaw_incremental_;

    switch (last_key_) {
      case 'w': cmd.linear.x  =  linear;   break;
      case 's': cmd.linear.x  = -linear;   break;
      case 'a': cmd.linear.y  =  linear;   break;
      case 'd': cmd.linear.y  = -linear;   break;
      case 'r': cmd.linear.z  =  vertical; break;
      case 'f': cmd.linear.z  = -vertical; break;
      case 'q': cmd.angular.z =  yaw_v;    break;
      case 'e': cmd.angular.z = -yaw_v;    break;
      case ' ':
      case 'z': break;  // cmd queda a cero → nivel
      default:  break;
    }
    return cmd;
  }

  // ── Loop principal ───────────────────────────────────────────────────────────

  void loop()
  {
    const char key = read_key();
    handle_key(key);

    const geometry_msgs::msg::Twist cmd = build_cmd_from_last_key();
    cmd_pub_->publish(cmd);
  }

  // ── Constantes ───────────────────────────────────────────────────────────────

  static constexpr double YAW_STEP_BASE  = 0.045;
  static constexpr double INCREMENT_STEP = 0.10;

  // ── Parámetros ───────────────────────────────────────────────────────────────

  double publish_hz_{30.0};
  double linear_speed_{0.75};
  double vertical_speed_{0.35};
  double key_timeout_s_{0.20};
  double linear_incremental_{1.0};
  double vertical_incremental_{1.0};
  double yaw_incremental_{1.0};

  // ── Estado ───────────────────────────────────────────────────────────────────

  char last_key_{0};
  rclcpp::Time last_key_time_;

  bool termios_available_{false};
  struct termios original_termios_ {};

  // ── ROS ──────────────────────────────────────────────────────────────────────

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     key_pub_;
  rclcpp::TimerBase::SharedPtr                            timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardNode>());
  rclcpp::shutdown();
  return 0;
}
