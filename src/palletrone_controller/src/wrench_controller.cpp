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
    const double KP_ATT[3] = {1.50, 1.50, 1.50};
    const double KI_ATT[3] = {0, 0, 0};
    const double KD_ATT[3] = {1.20, 1.20, 1.20};
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
    // 실제 compensation에 사용된 외란 보상값을 디버깅용으로 publish한다.
    pub_external_wrench_comp_ = this->create_publisher<palletrone_interfaces::msg::Wrench>(
      "/external_wrench_comp", 10);
    sub_att_cmd_ = this->create_subscription<palletrone_interfaces::msg::AttitudeCmd>(
      "/att_cmd", 10, std::bind(&WrenchController::onAttCmd, this, std::placeholders::_1)); //추가
    // wrench_observer가 추정한 외란 wrench. 이 값은 바로 보상하지 않고 아래 safety layer를 거친다.
    sub_external_wrench_hat_ = this->create_subscription<palletrone_interfaces::msg::Wrench>(
      "/external_wrench_hat", 10, std::bind(&WrenchController::onExternalWrenchHat, this, std::placeholders::_1));

    // 추정 force 중 실제 보상에 반영할 축별 gain. 기본값은 force 보상 전체 off.
    comp_force_gain_ = declareVector3Parameter("external_wrench_comp_force_gain", {0.0, 0.0, 0.0});
    // 추정 moment 중 실제 보상에 반영할 축별 gain. 현재 기본값은 Mx/My/Mz를 각각 90% 보상.
    comp_moment_gain_ = declareVector3Parameter("external_wrench_comp_moment_gain", {0.9, 0.9, 0.9});
    // 보상 force/moment의 축별 최대 절댓값. observer 추정값이 튀어도 이 값 이상 보상하지 않는다.
    max_force_comp_ = declareVector3Parameter("external_wrench_max_force_comp", {5.0, 5.0, 5.0});
    max_moment_comp_ = declareVector3Parameter("external_wrench_max_moment_comp", {0.3, 0.3, 0.3});
    // 보상값의 축별 변화율 제한. step 형태로 보상이 들어가 actuator를 때리는 것을 막는다.
    max_force_comp_rate_ = declareVector3Parameter("external_wrench_max_force_comp_rate", {2.0, 2.0, 2.0});
    max_moment_comp_rate_ = declareVector3Parameter("external_wrench_max_moment_comp_rate", {0.05, 0.05, 0.05});
    // observer 메시지가 이 시간 이상 끊기면 보상 target을 0으로 ramp-down한다.
    external_wrench_timeout_ = this->declare_parameter<double>("external_wrench_timeout", 0.2);
    // 최종 wrench가 대략 actuator가 감당 가능한 범위인지 검사해서 보상량을 줄이는 safety switch.
    feasibility_check_enabled_ = this->declare_parameter<bool>("external_wrench_feasibility_check_enabled", true);
    // allocator 앞단에서 쓰는 간단한 feasible wrench proxy 한계값들.
    max_feasible_force_xy_ = this->declare_parameter<double>("external_wrench_max_feasible_force_xy", 15.0);
    max_feasible_force_z_ = this->declare_parameter<double>("external_wrench_max_feasible_force_z", 120.0);
    max_feasible_moment_xy_ = this->declare_parameter<double>("external_wrench_max_feasible_moment_xy", 1.0);
    max_feasible_moment_z_ = this->declare_parameter<double>("external_wrench_max_feasible_moment_z", 2.0);

    // /cmd가 오기 전 위치 목표와 /att_cmd가 오기 전 yaw 목표는 첫 state에서 현재값으로 latch한다.
    // 이렇게 해야 startup 때 원점/영 yaw로 갑자기 끌려가지 않는다.
    pos_cmd_.setZero();
    last_time_ = this->now();
  }

private:
  // ROS parameter로 길이 3짜리 double array를 선언하고 Eigen::Vector3d로 변환한다.
  // 사용자가 길이를 짧게 넣으면 부족한 축은 defaults 값을 사용한다.
  Eigen::Vector3d declareVector3Parameter(
    const std::string & name,
    const std::vector<double> & defaults)
  {
    const std::vector<double> values = this->declare_parameter<std::vector<double>>(name, defaults);
    Eigen::Vector3d out;
    for (int i = 0; i < 3; ++i) {
      out(i) = i < static_cast<int>(values.size()) ? values[i] : defaults[i];
    }
    return out;
  }

  void onCmd(const palletrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    // 위치 명령 [x, y, z]만 저장한다.
    // 제어 계산은 /palletrone_state callback에서만 수행해 PID dt를 state 주기에 맞춘다.
    pos_cmd_ <<
      static_cast<double>(msg->pos_cmd[0]),
      static_cast<double>(msg->pos_cmd[1]),
      static_cast<double>(msg->pos_cmd[2]);
    have_cmd_ = true;
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
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
  {
    // plant/estimator가 publish한 현재 위치, 속도, 자세, body angular rate를 저장한다.
    pos_ << static_cast<double>(msg->pos[0]), static_cast<double>(msg->pos[1]), static_cast<double>(msg->pos[2]); 
    vel_ << static_cast<double>(msg->vel[0]), static_cast<double>(msg->vel[1]), static_cast<double>(msg->vel[2]); 
    rpy_ << static_cast<double>(msg->rpy[0]), static_cast<double>(msg->rpy[1]), static_cast<double>(msg->rpy[2]); 
    w_body_ << static_cast<double>(msg->w_rpy[0]), static_cast<double>(msg->w_rpy[1]), static_cast<double>(msg->w_rpy[2]); 

    if (!have_state_) {
      // 첫 state를 받는 순간 현재 위치를 기본 position reference로 잡는다.
      // 이후 /cmd가 오면 onCmd()에서 pos_cmd_가 새 명령으로 덮인다.
      if (!have_cmd_) {
        pos_cmd_ = pos_;
      }

      // /att_cmd가 아직 없으면 roll/pitch는 수평(0), yaw는 현재 yaw를 유지한다.
      // yaw까지 0으로 두면 시작 순간 원치 않는 yaw 회전 명령이 생길 수 있다.
      if (!have_att_cmd_) {
        att_cmd_ << 0.0, 0.0, rpy_.z();
      }

      // 첫 제어 dt가 node 생성 시각부터 계산되지 않도록 state 수신 시각으로 기준을 리셋한다.
      last_time_ = this->now();
    }

    have_state_ = true;
    // 제어 계산과 /wrench publish는 state callback 한 군데에서만 수행한다.
    tryPublish();
  }

  void onExternalWrenchHat(const palletrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    // MoB observer가 추정한 body-frame 외란 force/moment를 저장한다.
    // 이 값은 직접 빼지 않고 updateExternalWrenchCompensation()에서 gain/limit/ramp를 거친다.
    external_force_hat_body_ <<
      static_cast<double>(msg->force[0]),
      static_cast<double>(msg->force[1]),
      static_cast<double>(msg->force[2]);
    external_moment_hat_body_ <<
      static_cast<double>(msg->moment[0]),
      static_cast<double>(msg->moment[1]),
      static_cast<double>(msg->moment[2]);

    have_external_wrench_hat_ = true;
    last_external_wrench_hat_time_ = this->now();
  }

  void tryPublish()
  {
    if (!have_state_) return;

    // state callback 간 실제 경과 시간을 PID dt로 사용한다.
    // tryPublish()는 onState()에서만 호출되므로 dt가 controller의 제어주기 의미를 갖는다.
    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0/400.0;

    // /cmd가 없을 때도 첫 state에서 latch된 현재 위치를 reference로 사용한다.
    const Eigen::Vector3d pos_ref = pos_cmd_;
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

    // 1) 자세 참조 벡터 선택.
    // /att_cmd가 없을 때도 첫 state에서 yaw hold reference가 att_cmd_에 들어 있다.
    const Eigen::Vector3d att_ref = att_cmd_;

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

    // MoB 추정값을 실제 보상값으로 변환한다. gain, clamp, ramp, timeout, feasibility check가 여기서 적용된다.
    updateExternalWrenchCompensation(now, dt, F_body, M_body);

    // nominal wrench에서 보상 wrench를 뺀다. 외란 추정값 전체가 아니라 제한된 comp 값만 사용한다.
    if (have_external_wrench_comp_) {
      F_body -= external_force_comp_body_;
      M_body -= external_moment_comp_body_;
      F_body.z() = std::max(0.0, F_body.z());
    }
    // 실제로 적용된 compensation 값을 별도 토픽으로 내보내 tuning/debug에 사용한다.
    publishWrench(pub_external_wrench_comp_, external_force_comp_body_, external_moment_comp_body_);

    palletrone_interfaces::msg::Wrench w; 
    w.moment[0] = static_cast<float>(M_body(0)); 
    w.moment[1] = static_cast<float>(M_body(1)); 
    w.moment[2] = static_cast<float>(M_body(2)); 
    w.force[0] = static_cast<float>(F_body(0)); 
    w.force[1] = static_cast<float>(F_body(1)); 
    w.force[2] = static_cast<float>(F_body(2)); 
    
    pub_wrench_->publish(w);
  }

  Eigen::Vector3d clampAbsVector(
    const Eigen::Vector3d & value,
    const Eigen::Vector3d & limit) const
  {
    // 각 축을 [-limit, +limit] 범위로 제한한다.
    Eigen::Vector3d out;
    for (int i = 0; i < 3; ++i) {
      const double abs_limit = std::max(0.0, limit(i));
      out(i) = std::clamp(value(i), -abs_limit, abs_limit);
    }
    return out;
  }

  Eigen::Vector3d rampVector(
    const Eigen::Vector3d & current,
    const Eigen::Vector3d & target,
    const Eigen::Vector3d & rate_limit,
    double dt) const
  {
    // current가 target으로 갈 때 한 주기당 최대 변화량을 rate_limit * dt로 제한한다.
    Eigen::Vector3d out = current;
    for (int i = 0; i < 3; ++i) {
      const double step = std::max(0.0, rate_limit(i)) * std::max(0.0, dt);
      out(i) += std::clamp(target(i) - current(i), -step, step);
    }
    return out;
  }

  bool isFeasibleWrench(
    const Eigen::Vector3d & force_body,
    const Eigen::Vector3d & moment_body) const
  {
    if (!feasibility_check_enabled_) {
      return true;
    }

    // 정확한 allocator feasible set은 아니지만, 너무 큰 lateral force/moment 명령을 막기 위한 proxy 검사다.
    const double force_xy = std::hypot(force_body.x(), force_body.y());
    const double moment_xy = std::hypot(moment_body.x(), moment_body.y());
    return force_body.z() >= 0.0 &&
           force_body.z() <= max_feasible_force_z_ &&
           force_xy <= max_feasible_force_xy_ &&
           moment_xy <= max_feasible_moment_xy_ &&
           std::abs(moment_body.z()) <= max_feasible_moment_z_;
  }

  double compensationFeasibilityScale(
    const Eigen::Vector3d & nominal_force_body,
    const Eigen::Vector3d & nominal_moment_body,
    const Eigen::Vector3d & force_comp_body,
    const Eigen::Vector3d & moment_comp_body) const
  {
    // 보상값을 100% 적용한 최종 wrench가 feasible하면 그대로 사용한다.
    if (isFeasibleWrench(nominal_force_body - force_comp_body, nominal_moment_body - moment_comp_body)) {
      return 1.0;
    }

    // feasible하지 않으면 binary search로 보상 scale을 0~1 사이에서 줄인다.
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 20; ++i) {
      const double mid = 0.5 * (lo + hi);
      if (isFeasibleWrench(
          nominal_force_body - mid * force_comp_body,
          nominal_moment_body - mid * moment_comp_body)) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return lo;
  }

  void updateExternalWrenchCompensation(
    const rclcpp::Time & now,
    double dt,
    const Eigen::Vector3d & nominal_force_body,
    const Eigen::Vector3d & nominal_moment_body)
  {
    // observer 값이 너무 오래된 경우 마지막 추정값을 계속 쓰지 않고 target을 0으로 둔다.
    const bool hat_fresh =
      have_external_wrench_hat_ &&
      last_external_wrench_hat_time_.nanoseconds() > 0 &&
      (now - last_external_wrench_hat_time_).seconds() <= external_wrench_timeout_;

    Eigen::Vector3d force_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d moment_target = Eigen::Vector3d::Zero();
    if (hat_fresh) {
      // observer 추정값에 축별 gain을 곱해 compensation target을 만든다.
      force_target = comp_force_gain_.cwiseProduct(external_force_hat_body_);
      moment_target = comp_moment_gain_.cwiseProduct(external_moment_hat_body_);
      // compensation target이 설정된 최대 보상량을 넘지 않도록 제한한다.
      force_target = clampAbsVector(force_target, max_force_comp_);
      moment_target = clampAbsVector(moment_target, max_moment_comp_);
    }

    // target으로 바로 점프하지 않고 rate limit을 적용해 천천히 따라간다.
    external_force_comp_body_ = rampVector(
      external_force_comp_body_, force_target, max_force_comp_rate_, dt);
    external_moment_comp_body_ = rampVector(
      external_moment_comp_body_, moment_target, max_moment_comp_rate_, dt);

    // 보상을 적용한 최종 wrench가 feasible proxy를 넘으면 보상값 전체를 같은 비율로 줄인다.
    const double feasible_scale = compensationFeasibilityScale(
      nominal_force_body,
      nominal_moment_body,
      external_force_comp_body_,
      external_moment_comp_body_);
    external_force_comp_body_ *= feasible_scale;
    external_moment_comp_body_ *= feasible_scale;
    // 아주 작은 수치 오차만 남은 경우에는 보상 off 상태로 본다.
    have_external_wrench_comp_ =
      external_force_comp_body_.cwiseAbs().maxCoeff() > 1e-9 ||
      external_moment_comp_body_.cwiseAbs().maxCoeff() > 1e-9;
  }

  void publishWrench(
    const rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr & publisher,
    const Eigen::Vector3d & force_body,
    const Eigen::Vector3d & moment_body) const
  {
    // Eigen vector를 palletrone_interfaces/Wrench 메시지로 변환해 publish한다.
    palletrone_interfaces::msg::Wrench msg;
    msg.force[0] = static_cast<float>(force_body.x());
    msg.force[1] = static_cast<float>(force_body.y());
    msg.force[2] = static_cast<float>(force_body.z());
    msg.moment[0] = static_cast<float>(moment_body.x());
    msg.moment[1] = static_cast<float>(moment_body.y());
    msg.moment[2] = static_cast<float>(moment_body.z());
    publisher->publish(msg);
  }

  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_external_wrench_comp_;
  rclcpp::Subscription<palletrone_interfaces::msg::AttitudeCmd>::SharedPtr   sub_att_cmd_;  // ★ 추가
  rclcpp::Subscription<palletrone_interfaces::msg::Wrench>::SharedPtr sub_external_wrench_hat_;
  rclcpp::Time last_time_;
  rclcpp::Time last_fz_clamp_log_time_;
  rclcpp::Time last_external_wrench_hat_time_;

  // 명령/상태 저장 변수.
  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d att_cmd_{Eigen::Vector3d::Zero()};
  // observer가 추정한 외란 wrench와 controller가 실제로 적용할 compensation wrench.
  Eigen::Vector3d external_force_hat_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_moment_hat_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_comp_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_moment_comp_body_{Eigen::Vector3d::Zero()};
  // compensation tuning parameter 저장 변수.
  Eigen::Vector3d comp_force_gain_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d comp_moment_gain_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_force_comp_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_moment_comp_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_force_comp_rate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_moment_comp_rate_{Eigen::Vector3d::Zero()};
  
  std::function<double(double,double,double,double)> pid_pos_[3];
  std::function<double(double,double,double,double)> pid_att_[3];

  const double mass_{4.4}, grav_{9.81};
  double external_wrench_timeout_{0.2};
  double max_feasible_force_xy_{15.0};
  double max_feasible_force_z_{120.0};
  double max_feasible_moment_xy_{1.0};
  double max_feasible_moment_z_{2.0};
  bool feasibility_check_enabled_{true};
  // 각 입력 토픽을 최소 한 번 받았는지 나타내는 flag.
  bool have_state_{false}, have_cmd_{false};
  bool have_att_cmd_{false};
  bool have_external_wrench_hat_{false};
  bool have_external_wrench_comp_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}
