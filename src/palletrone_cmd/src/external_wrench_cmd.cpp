#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

class ExternalWrenchCmd : public rclcpp::Node
{
public:
  ExternalWrenchCmd()
  : rclcpp::Node("external_wrench_cmd")
  {
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<double>("force_step", 0.5);
    this->declare_parameter<double>("moment_step", 0.02);
    this->declare_parameter<double>("force_limit", .0);
    this->declare_parameter<double>("moment_limit", 2.0);

    double publish_hz = this->get_parameter("publish_hz").as_double();
    force_step_ = this->get_parameter("force_step").as_double();
    moment_step_ = this->get_parameter("moment_step").as_double();
    force_limit_ = this->get_parameter("force_limit").as_double();
    moment_limit_ = this->get_parameter("moment_limit").as_double();

    if (publish_hz <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_hz <= 0.0, using 50 Hz");
      publish_hz = 50.0;
    }

    setupTerminal();

    pub_wrench_ = this->create_publisher<palletrone_interfaces::msg::Wrench>("/external_wrench_cmd", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ExternalWrenchCmd::onTimer, this));

    printHelp();
    RCLCPP_INFO(
      this->get_logger(),
      "ExternalWrenchCmd started. force_step=%.3f N, moment_step=%.3f Nm, force_limit=%.3f N, moment_limit=%.3f Nm",
      force_step_, moment_step_, force_limit_, moment_limit_);
  }

  ~ExternalWrenchCmd() override
  {
    publishZeroCommand();
    restoreTerminal();
  }

private:
  void setupTerminal()
  {
    if (!isatty(STDIN_FILENO)) {
      input_fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK);
      if (input_fd_ == -1) {
        RCLCPP_WARN(this->get_logger(), "stdin is not a terminal and /dev/tty is unavailable; keyboard input is unavailable");
        input_fd_ = STDIN_FILENO;
        return;
      }
      close_input_fd_ = true;
    } else {
      input_fd_ = STDIN_FILENO;
    }

    if (tcgetattr(input_fd_, &original_termios_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(input_fd_, TCSANOW, &raw) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return;
    }

    original_flags_ = fcntl(input_fd_, F_GETFL, 0);
    if (original_flags_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "fcntl(F_GETFL) failed: %s", std::strerror(errno));
      return;
    }

    if (fcntl(input_fd_, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      RCLCPP_ERROR(this->get_logger(), "fcntl(F_SETFL) failed: %s", std::strerror(errno));
      return;
    }

    terminal_ready_ = true;
  }

  void restoreTerminal()
  {
    if (!terminal_ready_) {
      if (close_input_fd_) {
        close(input_fd_);
        close_input_fd_ = false;
        input_fd_ = STDIN_FILENO;
      }
      return;
    }

    (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
    (void)fcntl(input_fd_, F_SETFL, original_flags_);
    terminal_ready_ = false;
    if (close_input_fd_) {
      close(input_fd_);
      close_input_fd_ = false;
      input_fd_ = STDIN_FILENO;
    }
  }

  void printHelp() const
  {
    std::printf(
      "\n"
      "[external_wrench_cmd key map]\n"
      "  q/a: Mx +/-    w/s: My +/-    e/d: Mz +/-\n"
      "  i/k: Fx +/-    j/l: Fy +/-    u/o: Fz +/-\n"
      "  z: reset wrench to zero       x: quit node\n"
      "  wrench is body-frame and persists until changed or reset\n\n");
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
    while ((n = read(input_fd_, &ch, 1)) > 0) {
      switch (ch) {
        case 'q':
          increment(moment_[0], +moment_step_, moment_limit_, "Mx");
          break;
        case 'a':
          increment(moment_[0], -moment_step_, moment_limit_, "Mx");
          break;
        case 'w':
          increment(moment_[1], +moment_step_, moment_limit_, "My");
          break;
        case 's':
          increment(moment_[1], -moment_step_, moment_limit_, "My");
          break;
        case 'e':
          increment(moment_[2], +moment_step_, moment_limit_, "Mz");
          break;
        case 'd':
          increment(moment_[2], -moment_step_, moment_limit_, "Mz");
          break;
        case 'i':
          increment(force_[0], +force_step_, force_limit_, "Fx");
          break;
        case 'k':
          increment(force_[0], -force_step_, force_limit_, "Fx");
          break;
        case 'j':
          increment(force_[1], +force_step_, force_limit_, "Fy");
          break;
        case 'l':
          increment(force_[1], -force_step_, force_limit_, "Fy");
          break;
        case 'u':
          increment(force_[2], +force_step_, force_limit_, "Fz");
          break;
        case 'o':
          increment(force_[2], -force_step_, force_limit_, "Fz");
          break;
        case 'z':
          force_.fill(0.0);
          moment_.fill(0.0);
          RCLCPP_INFO(this->get_logger(), "external wrench reset to zero");
          break;
        case 'x':
          RCLCPP_INFO(this->get_logger(), "quit requested");
          force_.fill(0.0);
          moment_.fill(0.0);
          publishZeroCommand();
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
        "keyboard read failed: %s",
        std::strerror(errno));
    }
  }

  void increment(double & value, double delta, double limit, const char * name)
  {
    const double before = value;
    value = std::clamp(value + delta, -limit, limit);
    RCLCPP_INFO(this->get_logger(), "%s: %.3f -> %.3f", name, before, value);
  }

  void publishCommand()
  {
    palletrone_interfaces::msg::Wrench msg;
    for (size_t i = 0; i < 3; ++i) {
      msg.force[i] = static_cast<float>(force_[i]);
      msg.moment[i] = static_cast<float>(moment_[i]);
    }
    pub_wrench_->publish(msg);
  }

  void publishZeroCommand()
  {
    if (!pub_wrench_) {
      return;
    }

    palletrone_interfaces::msg::Wrench msg;
    for (size_t i = 0; i < 3; ++i) {
      msg.force[i] = 0.0f;
      msg.moment[i] = 0.0f;
    }

    for (int i = 0; i < 5; ++i) {
      pub_wrench_->publish(msg);
    }
  }

  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 3> force_{0.0, 0.0, 0.0};
  std::array<double, 3> moment_{0.0, 0.0, 0.0};

  double force_step_{0.5};
  double moment_step_{0.02};
  double force_limit_{3.0};
  double moment_limit_{0.2};

  bool terminal_ready_{false};
  bool close_input_fd_{false};
  int input_fd_{STDIN_FILENO};
  int original_flags_{0};
  termios original_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalWrenchCmd>());
  rclcpp::shutdown();
  return 0;
}
