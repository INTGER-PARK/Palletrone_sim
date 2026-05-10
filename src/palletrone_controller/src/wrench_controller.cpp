#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <palletrone_interfaces/msg/attitude_cmd.hpp>

namespace
{
// /att_cmd는 degree 단위로 들어오므로 controller 내부 radian 단위로 변환할 때 사용한다.
constexpr double kDegToRad = M_PI / 180.0;
}

class WrenchController : public rclcpp::Node
{
public:
  WrenchController() : rclcpp::Node("wrench_controller")
  {
    // 위치 PID gain. 출력은 world-frame 목표 force 성분으로 사용된다.
    const double KP_POS[3] = {28.0, 28.0, 20.0}; //3 3 6
    const double KI_POS[3] = {1.5, 1.5, 1.1}; //0.01 0.01 .0.01
    const double KD_POS[3] = {6.0, 6.0, 10.0};
    const double I_MIN_POS = -5.00, I_MAX_POS = 10, OUT_MIN_POS = -200.0, OUT_MAX_POS = 200.0;

    // 자세 PID gain. 출력은 body-frame 목표 moment 성분으로 사용된다.
    const double KP_ATT[3] = {2.00, 2.00, 2.00};
    const double KI_ATT[3] = {0.3, 0.3, 0.3};
    const double KD_ATT[3] = {1.30, 1.30, 1.30};
    const double I_MIN_ATT = -1.0, I_MAX_ATT = 1.0, OUT_MIN_ATT = -5.0, OUT_MAX_ATT = 5.0;

    // ref, cur, dcur, dt를 받아 PID 출력 하나를 계산하는 closure를 만든다.
    // dcur는 현재 값의 미분값이므로, derivative error는 ref_dot=0 가정에서 -dcur로 둔다.
    auto init_pid = [](double kp, double ki, double kd, double i_min, double i_max, double out_min, double out_max) -> std::function<double(double,double,double,double)>
    {
      double iacc = 0.0;
      return [=](double ref, double cur, double dcur, double dt) mutable
      {
        if (dt <= 0.0) dt = 1e-3;
        const double e = ref - cur;
        const double de = -dcur;
        iacc += ki * e * dt;
        iacc = std::clamp(iacc, i_min, i_max);
        double u = kp*e + iacc + kd*de;
        return std::clamp(u, out_min, out_max);
      };
    };

    // 위치 제어용 PID 3축 초기화: x, y, z.
    pid_pos_[0] = init_pid(KP_POS[0], KI_POS[0], KD_POS[0], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);
    pid_pos_[1] = init_pid(KP_POS[1], KI_POS[1], KD_POS[1], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);
    pid_pos_[2] = init_pid(KP_POS[2], KI_POS[2], KD_POS[2], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);

    // 자세 제어용 PID 3축 초기화: roll, pitch, yaw.
    pid_att_[0] = init_pid(KP_ATT[0], KI_ATT[0], KD_ATT[0], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);
    pid_att_[1] = init_pid(KP_ATT[1], KI_ATT[1], KD_ATT[1], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);
    pid_att_[2] = init_pid(KP_ATT[2], KI_ATT[2], KD_ATT[2], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);

    // /cmd: 위치 명령, /palletrone_state: 현재 상태, /att_cmd: 자세 명령을 받는다.
    // /wrench는 allocator_controller가 사용할 최종 force/moment 명령이다.
    sub_cmd_ = this->create_subscription<palletrone_interfaces::msg::Cmd>("/cmd", 10, std::bind(&WrenchController::onCmd, this, std::placeholders::_1));
    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>("/palletrone_state", 10, std::bind(&WrenchController::onState, this, std::placeholders::_1));
    pub_wrench_ = this->create_publisher<palletrone_interfaces::msg::Wrench>("/wrench", 10);
    sub_att_cmd_ = this->create_subscription<palletrone_interfaces::msg::AttitudeCmd>(
      "/att_cmd", 10, std::bind(&WrenchController::onAttCmd, this, std::placeholders::_1)); //추가

    // /cmd가 오기 전 초기 위치 목표는 원점으로 둔다.
    pos_cmd_.setZero();
    // dt 계산을 위한 기준 시간 초기화.
    last_time_ = this->now();
  }

private:
  void onCmd(const palletrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    // 위치 명령 [x, y, z]를 저장하고, 상태가 있으면 바로 wrench를 갱신한다.
    pos_cmd_ << static_cast<double>(msg->pos_cmd[0]), static_cast<double>(msg->pos_cmd[1]), static_cast<double>(msg->pos_cmd[2]); have_cmd_ = true; tryPublish();
  }
  void onAttCmd(const palletrone_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    // AttitudeCmd.msg:
    //   float32 roll_ref
    //   float32 pitch_ref
    //   float32 yaw_ref

    // /att_cmd is published in degrees; convert once at the boundary.
    att_cmd_ << static_cast<double>(msg->roll_ref) * kDegToRad,
                static_cast<double>(msg->pitch_ref) * kDegToRad,
                static_cast<double>(msg->yaw_ref) * kDegToRad;

    have_att_cmd_ = true;

    // 상태가 이미 들어와 있다면 바로 /wrench 갱신
    tryPublish();
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
  {
    // plant/estimator가 publish한 현재 위치, 속도, 자세, body angular rate를 저장한다.
    pos_ << static_cast<double>(msg->pos[0]), static_cast<double>(msg->pos[1]), static_cast<double>(msg->pos[2]); 
    vel_ << static_cast<double>(msg->vel[0]), static_cast<double>(msg->vel[1]), static_cast<double>(msg->vel[2]); 
    rpy_ << static_cast<double>(msg->rpy[0]), static_cast<double>(msg->rpy[1]), static_cast<double>(msg->rpy[2]); 
    w_body_ << static_cast<double>(msg->w_rpy[0]), static_cast<double>(msg->w_rpy[1]), static_cast<double>(msg->w_rpy[2]); 
    
    have_state_ = true; tryPublish();
  }

  void tryPublish()
  {
    if (!have_state_) return;

    // callback 간 실제 경과 시간을 PID dt로 사용한다. 비정상적으로 크거나 작으면 400 Hz 기준값으로 보정한다.
    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0/400.0;

    // 위치 명령이 있으면 그 값을, 없으면 원점을 위치 reference로 사용한다.
    const Eigen::Vector3d pos_ref = have_cmd_ ? pos_cmd_ : Eigen::Vector3d::Zero();
    Eigen::Vector3d position_pid;
    position_pid.x() = pid_pos_[0](pos_ref.x(), pos_.x(), vel_.x(), dt);
    position_pid.y() = pid_pos_[1](pos_ref.y(), pos_.y(), vel_.y(), dt);
    position_pid.z() = pid_pos_[2](pos_ref.z(), pos_.z(), vel_.z(), dt);

    Eigen::Vector3d F_world; 
    F_world.x() = position_pid.x(); 
    F_world.y() = position_pid.y(); 
    // z축은 위치 PID 출력에 hover thrust에 해당하는 mg를 feed-forward로 더한다.
    F_world.z() = position_pid.z() + mass_ * grav_;

    // 현재 roll/pitch/yaw로 body-to-world 회전행렬 R_WB를 만든다.
    const double r = rpy_.x(), p = rpy_.y(), y = rpy_.z();
    const double sr = std::sin(r), cr = std::cos(r), sp = std::sin(p), cp = std::cos(p), sy = std::sin(y), cy = std::cos(y);

    Eigen::Matrix3d R_WB;
    R_WB <<  cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
               -sp,              cp*sr,              cp*cr;

    // allocator는 body-frame wrench를 받으므로 world-frame force를 body-frame으로 변환한다.
    Eigen::Vector3d F_body = R_WB.transpose() * F_world;
    const double raw_fz_body = F_body.z();
    // 이 모델에서 body z force는 음수가 되면 물리적으로 thrust로 만들 수 없으므로 0으로 제한한다.
    F_body.z() = std::max(0.0, F_body.z());

    if (raw_fz_body < 0.0) {
      const rclcpp::Time log_now = this->now();
      if (last_fz_clamp_log_time_.nanoseconds() == 0 ||
          (log_now - last_fz_clamp_log_time_).seconds() >= 0.2) {
        last_fz_clamp_log_time_ = log_now;
        RCLCPP_WARN(
          this->get_logger(),
          "wrench z clamp: F_world=[%.2f %.2f %.2f] F_body_raw=[%.2f %.2f %.2f] rpy_deg=[%.2f %.2f %.2f]",
          F_world.x(), F_world.y(), F_world.z(),
          F_body.x(), F_body.y(), raw_fz_body,
          rpy_.x() * 180.0 / M_PI, rpy_.y() * 180.0 / M_PI, rpy_.z() * 180.0 / M_PI);
      }
    }

    // 1) 자세 참조 벡터 선택: /att_cmd 가 없으면 [0,0,0]
    Eigen::Vector3d att_ref = have_att_cmd_ ? att_cmd_
                                            : Eigen::Vector3d::Zero();

    const double r_ref_d = att_ref.x();
    const double p_ref_d = att_ref.y();
    const double y_ref_d = att_ref.z();

    // 2) yaw wrap-around 처리
    const double y_err = std::atan2(std::sin(y_ref_d - rpy_.z()),
                                    std::cos(y_ref_d - rpy_.z()));
    const double y_ref_equiv = rpy_.z() + y_err;

    // 3) attitude PID
    Eigen::Vector3d M_body;
    M_body.x() = pid_att_[0](r_ref_d, rpy_.x(), w_body_.x(), dt);
    M_body.y() = pid_att_[1](p_ref_d, rpy_.y(), w_body_.y(), dt);
    M_body.z() = pid_att_[2](y_ref_equiv, rpy_.z(), w_body_.z(), dt);

    palletrone_interfaces::msg::Wrench w; 
    w.moment[0] = static_cast<float>(M_body(0)); 
    w.moment[1] = static_cast<float>(M_body(1)); 
    w.moment[2] = static_cast<float>(M_body(2)); 
    w.force[0] = static_cast<float>(F_body(0)); 
    w.force[1] = static_cast<float>(F_body(1)); 
    w.force[2] = static_cast<float>(F_body(2)); 
    
    pub_wrench_->publish(w);
  }

  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Subscription<palletrone_interfaces::msg::AttitudeCmd>::SharedPtr   sub_att_cmd_;  // ★ 추가
  rclcpp::Time last_time_;
  rclcpp::Time last_fz_clamp_log_time_;

  // 명령/상태 저장 변수.
  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d att_cmd_{Eigen::Vector3d::Zero()};

  std::function<double(double,double,double,double)> pid_pos_[3];
  std::function<double(double,double,double,double)> pid_att_[3];

  const double mass_{4.0}, grav_{9.81};
  // 각 입력 토픽을 최소 한 번 받았는지 나타내는 flag.
  bool have_state_{true}, have_cmd_{true};
  bool have_att_cmd_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}
