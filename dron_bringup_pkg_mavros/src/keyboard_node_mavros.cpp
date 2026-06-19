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
Modo manual por teclado - PX4 Offboard hibrido
----------------------------------------------
b : preparar/activar Offboard
m : armar
t : despegar en Offboard por posicion
n : desarmar
l : aterrizar con PX4 NAV_LAND

Control manual por intencion momentanea:
w/s : avanzar / retroceder
a/d : izquierda / derecha
r/f : subir / bajar
q/e : guiñada izquierda / derecha por pasos
espacio o z : parar inmediatamente

+ : aumentar velocidad incremental
- : disminuir velocidad incremental

p : imprimir estado en manual_control_node
h : ayuda
Ctrl+C : salir

)HELP";
}  // namespace

class KeyboardNode : public rclcpp::Node
{
public:
  KeyboardNode()
  : Node("keyboard_node_mavros")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 30.0);

    linear_speed_ = declare_parameter<double>("linear_speed", 0.75);
    vertical_speed_ = declare_parameter<double>("vertical_speed", 0.35);

    boost_linear_speed_ = declare_parameter<double>("boost_linear_speed", 1.25);
    boost_vertical_speed_ = declare_parameter<double>("boost_vertical_speed", 0.60);

    key_timeout_s_ = declare_parameter<double>("key_timeout_s", 0.20);

    linear_incremental_ = declare_parameter<double>("linear_incremental", 1.0);
    vertical_incremental_ = declare_parameter<double>("vertical_incremental", 1.0);
    yaw_incremental_ = declare_parameter<double>("yaw_incremental", 1.0);

    increment_step_ = 0.1;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/manual/cmd_vel", 10);
    key_pub_ = create_publisher<std_msgs::msg::String>("/manual/key", 10);

    last_key_time_ = now();

    if (tcgetattr(STDIN_FILENO, &original_termios_) == 0) {
      termios_available_ = true;
      auto raw = original_termios_;
      raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
      fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KeyboardNode::loop, this));

    RCLCPP_INFO(get_logger(), "%s", HELP);
  }

  ~KeyboardNode() override { restore_terminal(); }

private:
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

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
    if (result > 0 && FD_ISSET(STDIN_FILENO, &set)) {
      char c = 0;
      if (read(STDIN_FILENO, &c, 1) == 1)
        return c;
    }
    return 0;
  }

  void publish_key(char key)
  {
    std_msgs::msg::String msg;
    msg.data = std::string(1, key);
    key_pub_->publish(msg);
  }

  void handle_key(char key)
  {
    if (key == 0) return;
    if (key == 3) { rclcpp::shutdown(); return; }
    if (key == 'h') { RCLCPP_INFO(get_logger(), "%s", HELP); }

    // Incremento/descremento de velocidad
    if (key == '+') {
      linear_incremental_ += increment_step_;
      vertical_incremental_ += increment_step_;
      yaw_incremental_ += increment_step_;
      RCLCPP_INFO(get_logger(),
                  "Velocidades incrementadas: linear=%.2f vertical=%.2f yaw=%.2f",
                  linear_incremental_, vertical_incremental_, yaw_incremental_);
      return;
    }
    if (key == '-') {
      linear_incremental_ = std::max(0.1, linear_incremental_ - increment_step_);
      vertical_incremental_ = std::max(0.1, vertical_incremental_ - increment_step_);
      yaw_incremental_ = std::max(0.01, yaw_incremental_ - increment_step_);
      RCLCPP_INFO(get_logger(),
                  "Velocidades disminuidas: linear=%.2f vertical=%.2f yaw=%.2f",
                  linear_incremental_, vertical_incremental_, yaw_incremental_);
      return;
    }

    if (is_motion_key(key)) {
      last_key_ = key;
      last_key_time_ = now();
    }

    if (is_motion_key(key) || is_command_key(key))
      publish_key(key);
  }

  bool is_motion_key(char key) const
  {
    return key == 'w' || key == 's' || key == 'a' || key == 'd' ||
           key == 'r' || key == 'f' || key == ' ' || key == 'z';
  }

  bool is_command_key(char key) const
  {
    return key == 'b' || key == 'm' || key == 'n' || key == 't' ||
           key == 'l' || key == 'q' || key == 'e' || key == 'p' ||
           key == 'h' || key == '+' || key == '-';
  }

  bool key_is_recent() const
  {
    return (now() - last_key_time_).seconds() <= key_timeout_s_;
  }

  geometry_msgs::msg::Twist build_cmd_from_last_key()
  {
    geometry_msgs::msg::Twist cmd {};
    if (!key_is_recent()) return cmd;

    const double linear = linear_speed_ * linear_incremental_;
    const double vertical = vertical_speed_ * vertical_incremental_;
    const double yaw_scaled = yaw_step_ * yaw_incremental_;

    switch (last_key_) {
      case 'w': cmd.linear.x = linear; break;
      case 's': cmd.linear.x = -linear; break;
      case 'a': cmd.linear.y = linear; break;
      case 'd': cmd.linear.y = -linear; break;
      case 'r': cmd.linear.z = vertical; break;
      case 'f': cmd.linear.z = -vertical; break;
      case 'q': cmd.angular.z = yaw_scaled; break;
      case 'e': cmd.angular.z = -yaw_scaled; break;
      case ' ':
      case 'z': break;
    }

    return cmd;
  }

  void loop()
  {
    char key = read_key();
    handle_key(key);
    geometry_msgs::msg::Twist cmd = build_cmd_from_last_key();
    cmd_pub_->publish(cmd);
  }

  // --- Miembros ---
  double publish_hz_{30.0};
  double linear_speed_{0.75}, vertical_speed_{0.35}, yaw_step_{0.045};
  double boost_linear_speed_{1.25}, boost_vertical_speed_{0.60};
  double linear_incremental_{1.0}, vertical_incremental_{1.0}, yaw_incremental_{1.0};
  double key_timeout_s_{0.20};
  double increment_step_{0.1};

  bool boost_enabled_{false};
  char last_key_{0};
  rclcpp::Time last_key_time_;

  bool termios_available_{false};
  struct termios original_termios_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardNode>());
  rclcpp::shutdown();
  return 0;
}