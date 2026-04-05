#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/attitude_cmd.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kStepDeg = 0.5;
}  // namespace

class AttitudeSweepCmd : public rclcpp::Node
{
public:
  AttitudeSweepCmd()
  : rclcpp::Node("attitude_sweep_cmd")
  {
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<double>("max_abs_deg", 60.0);

    double publish_hz = this->get_parameter("publish_hz").as_double();
    max_abs_deg_ = this->get_parameter("max_abs_deg").as_double();

    if (publish_hz <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_hz <= 0.0 이라서 50Hz로 강제 설정합니다.");
      publish_hz = 50.0;
    }
    if (max_abs_deg_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "max_abs_deg <= 0.0 이라서 각도를 강제 설정합니다.");
      max_abs_deg_ = 60.0;
    }

    setupTerminal();

    pub_att_cmd_ = this->create_publisher<palletrone_interfaces::msg::AttitudeCmd>("/att_cmd", 10);

    auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AttitudeSweepCmd::onTimer, this));

    printHelp();
    RCLCPP_INFO(
      this->get_logger(),
      "AttitudeSweepCmd started. step=%.1f deg, max_abs_deg=%.1f, publish_hz=%.1f",
      kStepDeg, max_abs_deg_, publish_hz);
  }

  ~AttitudeSweepCmd() override
  {
    restoreTerminal();
  }

private:
  void setupTerminal()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(
        this->get_logger(),
        "stdin이 터미널이 아닙니다. 키 입력을 받을 수 없습니다.");
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr 실패: %s", std::strerror(errno));
      return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr 실패: %s", std::strerror(errno));
      return;
    }

    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "fcntl(F_GETFL) 실패: %s", std::strerror(errno));
      return;
    }

    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      RCLCPP_ERROR(this->get_logger(), "fcntl(F_SETFL) 실패: %s", std::strerror(errno));
      return;
    }

    terminal_ready_ = true;
  }

  void restoreTerminal()
  {
    if (!terminal_ready_) {
      return;
    }

    (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    (void)fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    terminal_ready_ = false;
  }

  void printHelp() const
  {
    std::printf(
      "\n"
      "[attitude_sweep_cmd key map]\n"
      "  q: roll  +0.5 deg    w: roll  -0.5 deg\n"
      "  e: pitch +0.5 deg    r: pitch -0.5 deg\n"
      "  t: yaw   +0.5 deg    y: yaw   -0.5 deg\n"
      "  z: reset all angles  x: quit node\n"
      "  limit: +/- %.1f deg\n\n",
      max_abs_deg_);
    std::fflush(stdout);
  }

  void onTimer()
  {
    processKeyboardInput();
    publishCommand();
  }

  void processKeyboardInput()
  {
    if (!terminal_ready_) {
      return;
    }

    char ch = 0;
    ssize_t n = 0;
    while ((n = read(STDIN_FILENO, &ch, 1)) > 0) {
      switch (ch) {
        case 'q':
          incrementAxis(roll_deg_, +kStepDeg, "roll");
          break;
        case 'w':
          incrementAxis(roll_deg_, -kStepDeg, "roll");
          break;
        case 'e':
          incrementAxis(pitch_deg_, +kStepDeg, "pitch");
          break;
        case 'r':
          incrementAxis(pitch_deg_, -kStepDeg, "pitch");
          break;
        case 't':
          incrementAxis(yaw_deg_, +kStepDeg, "yaw");
          break;
        case 'y':
          incrementAxis(yaw_deg_, -kStepDeg, "yaw");
          break;
        case 'z':
          roll_deg_ = 0.0;
          pitch_deg_ = 0.0;
          yaw_deg_ = 0.0;
          RCLCPP_INFO(this->get_logger(), "attitude reset to zero");
          break;
        case 'x':
          RCLCPP_INFO(this->get_logger(), "quit requested");
          rclcpp::shutdown();
          return;
        default:
          break;
      }
    }

    if (n == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "keyboard read 실패: %s",
        std::strerror(errno));
    }
  }

  void incrementAxis(double & axis_deg, double delta_deg, const char * axis_name)
  {
    const double before = axis_deg;
    axis_deg = std::clamp(axis_deg + delta_deg, -max_abs_deg_, max_abs_deg_);

    RCLCPP_INFO(
      this->get_logger(),
      "%s: %.1f deg -> %.1f deg",
      axis_name,
      before,
      axis_deg);
  }

  void publishCommand()
  {
    palletrone_interfaces::msg::AttitudeCmd msg;
    msg.roll_ref = static_cast<float>(roll_deg_);
    msg.pitch_ref = static_cast<float>(pitch_deg_);
    msg.yaw_ref = static_cast<float>(yaw_deg_);
    pub_att_cmd_->publish(msg);
  }

  rclcpp::Publisher<palletrone_interfaces::msg::AttitudeCmd>::SharedPtr pub_att_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_abs_deg_ {60.0};
  double roll_deg_ {0.0};
  double pitch_deg_ {0.0};
  double yaw_deg_ {0.0};

  bool terminal_ready_ {false};
  int original_flags_ {0};
  termios original_termios_ {};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttitudeSweepCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
