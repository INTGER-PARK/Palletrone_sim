#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/mob_observer_input.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>

class SecondWrenchObserver : public rclcpp::Node
{
public:
  // 2nd-order MoB natural frequency default.
  // Unit: rad/s. Example: 10.0 means 10 rad/s observer bandwidth.
  static constexpr double kDefaultOmegaN = 10.0;
  // 2nd-order MoB damping ratio default.
  // Unit: dimensionless. 1.0 is critical damping.
  static constexpr double kDefaultDampingRatio = 1.5;

  SecondWrenchObserver()
  : rclcpp::Node("second_wrench_observer")
  {
    // Force observer natural frequency for x/y/z force channels.
    // Unit: rad/s. Larger values make force estimates faster but noisier.
    omega_n_force_.x() = this->declare_parameter<double>("omega_n_force_x", kDefaultOmegaN);
    omega_n_force_.y() = this->declare_parameter<double>("omega_n_force_y", kDefaultOmegaN);
    omega_n_force_.z() = this->declare_parameter<double>("omega_n_force_z", kDefaultOmegaN);
    // Moment observer natural frequency for roll/pitch/yaw torque channels.
    // Unit: rad/s. Larger values make torque estimates faster but noisier.
    omega_n_moment_.x() = this->declare_parameter<double>("omega_n_moment_x", kDefaultOmegaN);
    omega_n_moment_.y() = this->declare_parameter<double>("omega_n_moment_y", kDefaultOmegaN);
    omega_n_moment_.z() = this->declare_parameter<double>("omega_n_moment_z", kDefaultOmegaN);
    // Force observer damping ratio for x/y/z channels.
    // Unit: dimensionless. 0.7 is underdamped, 1.0 critical, >1 overdamped.
    zeta_force_.x() = this->declare_parameter<double>("zeta_force_x", kDefaultDampingRatio);
    zeta_force_.y() = this->declare_parameter<double>("zeta_force_y", kDefaultDampingRatio);
    zeta_force_.z() = this->declare_parameter<double>("zeta_force_z", kDefaultDampingRatio);
    // Moment observer damping ratio for roll/pitch/yaw channels.
    // Unit: dimensionless.
    zeta_moment_.x() = this->declare_parameter<double>("zeta_moment_x", kDefaultDampingRatio);
    zeta_moment_.y() = this->declare_parameter<double>("zeta_moment_y", kDefaultDampingRatio);
    zeta_moment_.z() = this->declare_parameter<double>("zeta_moment_z", kDefaultDampingRatio);
    // Estimated external wrench output topic.
    // Unit of msg.force: N, body frame B. Unit of msg.moment: N*m, body frame B.
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/external_wrench_hat_second_order");

    // Rigid-body mass used for linear momentum p_W = m v_W.
    // Unit: kg.
    mass_ = this->declare_parameter<double>("mass", 4.0);
    // Gravity acceleration magnitude. gravity_world = [0, 0, -m*g].
    // Unit: m/s^2.
    gravity_ = this->declare_parameter<double>("gravity", 9.81);
    // Diagonal body inertia matrix entries.
    // Unit: kg*m^2.
    inertia_xx_ = this->declare_parameter<double>("inertia_xx", 0.360702);
    inertia_yy_ = this->declare_parameter<double>("inertia_yy", 0.360702);
    inertia_zz_ = this->declare_parameter<double>("inertia_zz", 0.660702);
    // Fixed simulation/state period used by the discrete observer update.
    // Unit: s. Default 1/400 s = 0.0025 s.
    state_dt_ = this->declare_parameter<double>("state_dt", 1.0 / 400.0);

    inertia_body_.setZero();
    inertia_body_(0, 0) = inertia_xx_;
    inertia_body_(1, 1) = inertia_yy_;
    inertia_body_(2, 2) = inertia_zz_;

    // State and actual actuator wrench input from plant.
    // The message is published once per mj_step() so state and known input share the same step.
    // pos [m], vel [m/s] in world frame W.
    // rpy [rad], w_rpy is body angular velocity omega_B [rad/s].
    // actuation_force [N] and actuation_moment [N*m] are in body frame B.
    sub_mob_observer_input_ = create_subscription<palletrone_interfaces::msg::MobObserverInput>(
      "/mob_observer_input",
      10,
      [this](palletrone_interfaces::msg::MobObserverInput::SharedPtr msg) {
        onMobObserverInput(msg);
      });

    pub_external_wrench_hat_ = create_publisher<palletrone_interfaces::msg::Wrench>(output_topic_, 10);
  }

private:
  void onMobObserverInput(const palletrone_interfaces::msg::MobObserverInput::SharedPtr msg)
  {
    // /mob_observer_input contains all MoB inputs from one mj_step().
    // This removes cross-topic callback ordering as a source of observer overshoot.
    const Eigen::Vector3d vel_world(
      static_cast<double>(msg->vel[0]),
      static_cast<double>(msg->vel[1]),
      static_cast<double>(msg->vel[2]));
    // roll/pitch/yaw attitude.
    // Unit: rad.
    const Eigen::Vector3d rpy(
      static_cast<double>(msg->rpy[0]),
      static_cast<double>(msg->rpy[1]),
      static_cast<double>(msg->rpy[2]));
    // Body angular velocity omega_B from MuJoCo gyro.
    // Unit: rad/s. This is not Euler angle rate.
    const Eigen::Vector3d omega_body(
      static_cast<double>(msg->w_rpy[0]),
      static_cast<double>(msg->w_rpy[1]),
      static_cast<double>(msg->w_rpy[2]));
    // Actuation force f_act_B.
    // Unit: N, body frame B.
    const Eigen::Vector3d actuation_force_body(
      static_cast<double>(msg->actuation_force[0]),
      static_cast<double>(msg->actuation_force[1]),
      static_cast<double>(msg->actuation_force[2]));
    // Actuation torque tau_act_B.
    // Unit: N*m, body frame B.
    const Eigen::Vector3d actuation_moment_body(
      static_cast<double>(msg->actuation_moment[0]),
      static_cast<double>(msg->actuation_moment[1]),
      static_cast<double>(msg->actuation_moment[2]));

    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy);

    // Momentum signals:
    //   p_W = m v_W.
    //     m [kg], v_W [m/s] -> p_W [kg*m/s] = [N*s], world frame W.
    //   h_B = J_B omega_B.
    //     J_B [kg*m^2], omega_B [rad/s] -> h_B [kg*m^2/s], body frame B.
    const Eigen::Vector3d linear_momentum_world = mass_ * vel_world;
    const Eigen::Vector3d angular_momentum_body = inertia_body_ * omega_body;

    if (!have_observer_state_) {
      // Initialize Q2(s)sP(s) with the current momentum to avoid startup impulse.
      force_filter_.reset(linear_momentum_world);
      moment_filter_.reset(angular_momentum_body);
      external_force_hat_world_.setZero();
      external_moment_hat_body_.setZero();
      have_observer_state_ = true;
      prev_sim_time_ = static_cast<double>(msg->sim_time);
      have_prev_time_ = true;
      return;
    }

    // Discrete integration period.
    // Unit: s. Prefer plant simulation time so skipped/delayed callbacks use the real step gap.
    double dt = state_dt_;
    if (have_prev_time_) {
      dt = static_cast<double>(msg->sim_time) - prev_sim_time_;
      dt = std::clamp(dt, 1e-6, 0.05);
    }
    prev_sim_time_ = static_cast<double>(msg->sim_time);
    have_prev_time_ = true;

    // Known input terms:
    //   u_f_W   = R_WB f_act_B + mg_W.
    //     f_act_B [N], mg_W [N] -> u_f_W [N], world frame W.
    //   u_tau_B = tau_act_B - omega_B x h_B.
    //     tau_act_B [N*m], omega_B x h_B [N*m] -> u_tau_B [N*m], body frame B.
    const Eigen::Vector3d gravity_world(0.0, 0.0, -mass_ * gravity_);
    const Eigen::Vector3d u_f_world =
      r_wb * actuation_force_body + gravity_world;
    const Eigen::Vector3d u_tau_body =
      actuation_moment_body - omega_body.cross(angular_momentum_body);

    // Second-order momentum observer:
    //   W_hat_ext(s) = Q2(s) [sP(s) - U(s)]
    //   Q2(s) = omega_n^2 / (s^2 + 2 zeta omega_n s + omega_n^2)
    //     omega_n [rad/s], zeta [-].
    //
    // Implemented without direct momentum differentiation:
    //   Q2(s)sP(s) - Q2(s)U(s).
    // Force result unit: [N]. Moment result unit: [N*m].
    external_force_hat_world_ = force_filter_.update(
      linear_momentum_world, u_f_world, omega_n_force_, zeta_force_, dt);
    external_moment_hat_body_ = moment_filter_.update(
      angular_momentum_body, u_tau_body, omega_n_moment_, zeta_moment_, dt);

    // Force observer runs in world frame W, but published Wrench force is body frame B.
    // external_force_hat_body unit: N.
    const Eigen::Vector3d external_force_hat_body = r_wb.transpose() * external_force_hat_world_;
    publishWrench(external_force_hat_body, external_moment_hat_body_);
  }

  Eigen::Matrix3d rotationWorldFromBody(Eigen::Vector3d rpy) const
  {
    // ZYX Euler rotation matrix R_WB.
    // Input rpy unit: rad. Output maps body-frame vector to world-frame vector.
    const double r = rpy.x();
    const double p = rpy.y();
    const double y = rpy.z();

    const double sr = std::sin(r);
    const double cr = std::cos(r);
    const double sp = std::sin(p);
    const double cp = std::cos(p);
    const double sy = std::sin(y);
    const double cy = std::cos(y);

    Eigen::Matrix3d r_wb;
    r_wb <<
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr;
    return r_wb;
  }

  void publishWrench(Eigen::Vector3d force_body, Eigen::Vector3d moment_body) const
  {
    // force_body unit: N, body frame B.
    // moment_body unit: N*m, body frame B.
    palletrone_interfaces::msg::Wrench msg;
    msg.force[0] = static_cast<float>(force_body.x());
    msg.force[1] = static_cast<float>(force_body.y());
    msg.force[2] = static_cast<float>(force_body.z());
    msg.moment[0] = static_cast<float>(moment_body.x());
    msg.moment[1] = static_cast<float>(moment_body.y());
    msg.moment[2] = static_cast<float>(moment_body.z());
    pub_external_wrench_hat_->publish(msg);
  }

  struct SecondOrderMobFilter
  {
    void reset(Eigen::Vector3d momentum)
    {
      // q2_p_state_ is Q2(s)P(s).
      // For force filter: [N*s]. For moment filter: [kg*m^2/s].
      q2_p_state_ = momentum;
      // q2_p_rate_ is d/dt Q2(s)P(s) = Q2(s)sP(s).
      // For force filter: [N]. For moment filter: [N*m].
      q2_p_rate_.setZero();
      // q2_u_state_ is Q2(s)U(s).
      // For force filter: [N]. For moment filter: [N*m].
      q2_u_state_.setZero();
      // q2_u_rate_ is d/dt Q2(s)U(s).
      // For force filter: [N/s]. For moment filter: [N*m/s].
      q2_u_rate_.setZero();
    }

    Eigen::Vector3d update(
      Eigen::Vector3d momentum,
      Eigen::Vector3d known_input,
      Eigen::Vector3d omega_n,
      Eigen::Vector3d zeta,
      double dt)
    {
      // momentum:
      //   force filter input p_W [N*s], moment filter input h_B [kg*m^2/s].
      // known_input:
      //   force filter input u_f_W [N], moment filter input u_tau_B [N*m].
      // omega_n: natural frequency [rad/s], e.g. 10 rad/s.
      // zeta: damping ratio [-].
      // dt: sample time [s].
      for (int i = 0; i < 3; ++i) {
        const double wn = std::max(0.0, omega_n(i));
        const double damping = std::max(0.0, zeta(i));
        const double wn2 = wn * wn;

        // 2nd-order low-pass driven by momentum P:
        //   x_p_ddot = wn^2(P - x_p) - 2*zeta*wn*x_p_dot
        // q2_p_rate_ = x_p_dot = Q2(s)sP(s).
        const double p_acc =
          wn2 * (momentum(i) - q2_p_state_(i)) -
          2.0 * damping * wn * q2_p_rate_(i);
        q2_p_rate_(i) += dt * p_acc;
        q2_p_state_(i) += dt * q2_p_rate_(i);

        // 2nd-order low-pass driven by known input U:
        //   x_u_ddot = wn^2(U - x_u) - 2*zeta*wn*x_u_dot
        // q2_u_state_ = x_u = Q2(s)U(s).
        const double u_acc =
          wn2 * (known_input(i) - q2_u_state_(i)) -
          2.0 * damping * wn * q2_u_rate_(i);
        q2_u_rate_(i) += dt * u_acc;
        q2_u_state_(i) += dt * q2_u_rate_(i);
      }

      // W_hat_ext = Q2(s)sP(s) - Q2(s)U(s).
      // For force filter: [N]. For moment filter: [N*m].
      return q2_p_rate_ - q2_u_state_;
    }

    Eigen::Vector3d q2_p_state_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d q2_p_rate_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d q2_u_state_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d q2_u_rate_{Eigen::Vector3d::Zero()};
  };

  rclcpp::Subscription<palletrone_interfaces::msg::MobObserverInput>::SharedPtr
    sub_mob_observer_input_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_external_wrench_hat_;

  std::string output_topic_{"/external_wrench_hat_second_order"};
  double mass_{4.0};
  double gravity_{9.81};
  double inertia_xx_{0.360702};
  double inertia_yy_{0.360702};
  double inertia_zz_{0.660702};
  double state_dt_{1.0 / 400.0};
  double prev_sim_time_{0.0};

  Eigen::Vector3d omega_n_force_{kDefaultOmegaN, kDefaultOmegaN, kDefaultOmegaN};
  Eigen::Vector3d omega_n_moment_{kDefaultOmegaN, kDefaultOmegaN, kDefaultOmegaN};
  Eigen::Vector3d zeta_force_{
    kDefaultDampingRatio,
    kDefaultDampingRatio,
    kDefaultDampingRatio};
  Eigen::Vector3d zeta_moment_{
    kDefaultDampingRatio,
    kDefaultDampingRatio,
    kDefaultDampingRatio};
  Eigen::Matrix3d inertia_body_{Eigen::Matrix3d::Zero()};

  SecondOrderMobFilter force_filter_;
  SecondOrderMobFilter moment_filter_;
  Eigen::Vector3d external_force_hat_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_moment_hat_body_{Eigen::Vector3d::Zero()};

  bool have_observer_state_{false};
  bool have_prev_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SecondWrenchObserver>());
  rclcpp::shutdown();
  return 0;
}
