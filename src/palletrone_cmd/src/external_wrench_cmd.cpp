#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

namespace
{
// 2 * Pi 값 정의. 사인파(Sine wave)의 각속도 계산을 위해 사용됩니다.
constexpr double kTwoPi = 6.28318530717958647692;
}

class ExternalWrenchCmd : public rclcpp::Node
{
public:
  ExternalWrenchCmd()
  : rclcpp::Node("external_wrench_cmd")
  {
    // ROS 파라미터 선언 및 기본값 설정
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<double>("force_step", 0.5);
    this->declare_parameter<double>("moment_step", 0.1);
    this->declare_parameter<double>("force_limit", 10.0);
    this->declare_parameter<double>("moment_limit", 5.0);
    // 사인파의 진폭(A) 기본값 설정
    this->declare_parameter<double>("moment_disturbance_amplitude", 0.3);
    // 사인파의 주파수(f, Hz) 기본값 설정
    this->declare_parameter<double>("moment_disturbance_frequency_hz", 0.5);

    // 파라미터 값을 멤버 변수로 가져오기
    double publish_hz = this->get_parameter("publish_hz").as_double();
    force_step_ = this->get_parameter("force_step").as_double();
    moment_step_ = this->get_parameter("moment_step").as_double();
    force_limit_ = this->get_parameter("force_limit").as_double();
    moment_limit_ = this->get_parameter("moment_limit").as_double();
    moment_disturbance_amplitude_ = this->get_parameter("moment_disturbance_amplitude").as_double();
    moment_disturbance_frequency_hz_ = this->get_parameter("moment_disturbance_frequency_hz").as_double();

    // 입력값 예외 처리 (음수가 들어오면 절댓값으로 변환)
    if (publish_hz <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_hz <= 0.0, using 50 Hz");
      publish_hz = 50.0;
    }
    if (moment_disturbance_amplitude_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "moment_disturbance_amplitude < 0.0, using absolute value");
      moment_disturbance_amplitude_ = std::abs(moment_disturbance_amplitude_);
    }
    if (moment_disturbance_frequency_hz_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "moment_disturbance_frequency_hz < 0.0, using absolute value");
      moment_disturbance_frequency_hz_ = std::abs(moment_disturbance_frequency_hz_);
    }

    // 터미널 키보드 입력을 비동기적으로 받기 위한 설정
    setupTerminal();

    // 외부 Wrench 명령을 퍼블리시할 퍼블리셔 생성
    pub_wrench_ = this->create_publisher<palletrone_interfaces::msg::Wrench>("/external_wrench_cmd", 10);

    // 설정한 publish_hz 주기에 맞춰 onTimer 함수를 반복 실행하는 타이머 설정
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ExternalWrenchCmd::onTimer, this));

    printHelp(); // 조작법 출력
    RCLCPP_INFO(
      this->get_logger(),
      "ExternalWrenchCmd started. force_step=%.3f N, moment_step=%.3f Nm, force_limit=%.3f N, moment_limit=%.3f Nm, disturbance=%.3f Nm @ %.2f Hz",
      force_step_, moment_step_, force_limit_, moment_limit_,
      moment_disturbance_amplitude_, moment_disturbance_frequency_hz_);
  }

  ~ExternalWrenchCmd() override
  {
    // 노드 종료 시 안전하게 힘/모멘트를 0으로 보내고 터미널 원상복구
    publishZeroCommand();
    restoreTerminal();
  }

private:
  void setupTerminal()
  {
    // 터미널 설정 (리눅스 환경에서 엔터를 누르지 않아도 즉각적으로 키 입력을 받기 위한 논블로킹 모드 셋업)
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
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO)); // 엔터 대기 해제, 입력된 글자 화면 출력 방지
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
    // 노드가 끝날 때 터미널 설정을 원래대로 되돌리는 함수
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
    // 조작법 안내 메시지
    std::printf(
      "\n"
      "[external_wrench_cmd key map]\n"
      "  q/a: Mx +/-    w/s: My +/-    e/d: Mz +/-\n"
      "  i/k: Fx +/-    j/l: Fy +/-    u: Fz +\n"
      "  o: toggle roll(Mx) sine disturbance\n"
      "  p: toggle pitch(My) sine disturbance\n"
      "  y: toggle yaw(Mz) sine disturbance\n"
      "  z: reset wrench to zero       x: quit node\n"
      "  wrench is body-frame and persists until changed or reset\n"
      "  disturbance: %.3f Nm @ %.2f Hz\n\n",
      moment_disturbance_amplitude_, moment_disturbance_frequency_hz_);
    std::fflush(stdout);
  }

  void onTimer()
  {
    // 지정된 주기(Hz)마다 반복 실행: 키보드 입력 체크 후 Wrench 데이터 퍼블리시
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
    // 입력된 키가 있는지 논블로킹으로 읽어옴
    while ((n = read(input_fd_, &ch, 1)) > 0) {
      switch (ch) {
        case 'q': increment(moment_[0], +moment_step_, moment_limit_, "Mx"); break; // Mx 증가
        case 'a': increment(moment_[0], -moment_step_, moment_limit_, "Mx"); break; // Mx 감소
        case 'w': increment(moment_[1], +moment_step_, moment_limit_, "My"); break;
        case 's': increment(moment_[1], -moment_step_, moment_limit_, "My"); break;
        case 'e': increment(moment_[2], +moment_step_, moment_limit_, "Mz"); break;
        case 'd': increment(moment_[2], -moment_step_, moment_limit_, "Mz"); break;
        case 'i': increment(force_[0], +force_step_, force_limit_, "Fx"); break;
        case 'k': increment(force_[0], -force_step_, force_limit_, "Fx"); break;
        case 'j': increment(force_[1], +force_step_, force_limit_, "Fy"); break;
        case 'l': increment(force_[1], -force_step_, force_limit_, "Fy"); break;
        case 'u': increment(force_[2], +force_step_, force_limit_, "Fz"); break;
        case 'o':
          // 'o'키를 누르면 Roll(Mx) 방향으로 사인파 외란(Disturbance)을 켜거나 끕니다.
          toggleDisturbance(roll_disturbance_enabled_, roll_disturbance_start_time_, "roll(Mx)");
          break;
        case 'p':
          // 'p'키를 누르면 Pitch(My) 방향으로 사인파 외란(Disturbance)을 켜거나 끕니다.
          toggleDisturbance(pitch_disturbance_enabled_, pitch_disturbance_start_time_, "pitch(My)");
          break;
        case 'y':
          // 'y'키를 누르면 Yaw(Mz) 방향으로 사인파 외란(Disturbance)을 켜거나 끕니다.
          toggleDisturbance(yaw_disturbance_enabled_, yaw_disturbance_start_time_, "yaw(Mz)");
          break;
        case 'z':
          // 'z'키를 누르면 모든 힘, 모멘트, 외란을 0으로 초기화
          force_.fill(0.0);
          moment_.fill(0.0);
          roll_disturbance_enabled_ = false;
          pitch_disturbance_enabled_ = false;
          yaw_disturbance_enabled_ = false;
          RCLCPP_INFO(this->get_logger(), "external wrench reset to zero");
          break;
        case 'x':
          // 'x'키를 누르면 노드 정상 종료
          RCLCPP_INFO(this->get_logger(), "quit requested");
          force_.fill(0.0);
          moment_.fill(0.0);
          roll_disturbance_enabled_ = false;
          pitch_disturbance_enabled_ = false;
          yaw_disturbance_enabled_ = false;
          publishZeroCommand();
          rclcpp::shutdown();
          return;
        default:
          break;
      }
    }

    if (n == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "keyboard read failed: %s", std::strerror(errno));
    }
  }

  void increment(double & value, double delta, double limit, const char * name)
  {
    // 입력값(증감)이 한계값(limit)을 넘지 않도록 자르는(clamping) 함수
    const double before = value;
    value = std::clamp(value + delta, -limit, limit);
    RCLCPP_INFO(this->get_logger(), "%s: %.3f -> %.3f", name, before, value);
  }

  void toggleDisturbance(bool & enabled, rclcpp::Time & start_time, const char * name)
  {
    // 사인파 외란 활성화/비활성화 스위치. 활성화될 때 시작 시간을 현재 시간으로 갱신하여 t=0부터 시작하게 함.
    enabled = !enabled;
    start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "%s sine disturbance %s", name, enabled ? "on" : "off");
  }

  double disturbanceMoment(bool enabled, const rclcpp::Time & start_time) const
  {
    // 실제 사인파 모멘트 값을 계산하는 함수
    if (!enabled || moment_disturbance_amplitude_ == 0.0 || moment_disturbance_frequency_hz_ == 0.0) {
      return 0.0;
    }

    // 외란이 켜진 후 경과한 시간(t) 계산
    const double elapsed = (this->now() - start_time).seconds();
    
    // y = A * sin(2 * Pi * f * t) 형태의 수식
    return moment_disturbance_amplitude_ * std::sin(kTwoPi * moment_disturbance_frequency_hz_ * elapsed);
  }

  void publishCommand()
  {
    palletrone_interfaces::msg::Wrench msg;
    for (size_t i = 0; i < 3; ++i) {
      msg.force[i] = static_cast<float>(force_[i]);
      msg.moment[i] = static_cast<float>(moment_[i]);
    }
    
    // Mx (Roll) 방향 모멘트에 사인파 외란 추가 후 리밋 제한
    msg.moment[0] = static_cast<float>(std::clamp(
      moment_[0] + disturbanceMoment(roll_disturbance_enabled_, roll_disturbance_start_time_),
      -moment_limit_,
      moment_limit_));
      
    // My (Pitch) 방향 모멘트에 사인파 외란 추가 후 리밋 제한
    msg.moment[1] = static_cast<float>(std::clamp(
      moment_[1] + disturbanceMoment(pitch_disturbance_enabled_, pitch_disturbance_start_time_),
      -moment_limit_,
      moment_limit_));

    // Mz (Yaw) 방향 모멘트에 사인파 외란 추가 후 리밋 제한
    msg.moment[2] = static_cast<float>(std::clamp(
      moment_[2] + disturbanceMoment(yaw_disturbance_enabled_, yaw_disturbance_start_time_),
      -moment_limit_,
      moment_limit_));
      
    pub_wrench_->publish(msg);
  }

  void publishZeroCommand()
  {
    // 종료 시나 초기화 시 힘과 모멘트를 완전히 0으로 만들어서 5번 연속 퍼블리시
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

  // ROS 2 통신 및 타이머 객체
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 기체에 가해지는 힘/모멘트 기본 상태
  std::array<double, 3> force_{0.0, 0.0, 0.0};
  std::array<double, 3> moment_{0.0, 0.0, 0.0};

  // 파라미터 변수들
  double force_step_{0.5};
  double moment_step_{0.02};
  double force_limit_{3.0};
  double moment_limit_{0.2};
  double moment_disturbance_amplitude_{0.3}; // 진폭
  double moment_disturbance_frequency_hz_{0.5}; // 주파수

  // 외란 상태 관리 변수들
  bool roll_disturbance_enabled_{false};
  bool pitch_disturbance_enabled_{false};
  bool yaw_disturbance_enabled_{false};
  rclcpp::Time roll_disturbance_start_time_{};
  rclcpp::Time pitch_disturbance_start_time_{};
  rclcpp::Time yaw_disturbance_start_time_{};

  // 터미널 제어용 변수
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
