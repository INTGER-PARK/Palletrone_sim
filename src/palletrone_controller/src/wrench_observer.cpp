#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/input.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>

class WrenchObserver : public rclcpp::Node
{
public:
  static constexpr double kDegToRad = M_PI / 180.0;

  // Tune these default K_I values first. They are diagonal observer gains [1/s].
  // Larger values make /external_wrench_hat follow the raw estimate faster.
  static constexpr double kDefaultKiForceX = 10.0;
  static constexpr double kDefaultKiForceY = 10.0;
  static constexpr double kDefaultKiForceZ = 10.0;
  static constexpr double kDefaultKiMomentX = 10.0;
  static constexpr double kDefaultKiMomentY = 10.0;
  static constexpr double kDefaultKiMomentZ = 10.0;

  WrenchObserver()
  : rclcpp::Node("wrench_observer")
  {
    // Model parameters assumed known by the observer.
    // There is no EKF or online parameter identification in this node.
    // The observer directly uses fixed rigid-body parameters provided as ROS parameters:
    //   mass, gravity, inertia_xx, inertia_yy, inertia_zz,
    //   thrust_coefficient, reaction_torque_coefficient,
    //   arm_radius, rotor_z_offset, com_x, com_y, com_z.
    //
    // Momentum-observer tuning parameters:
    //   k_i_force_x, k_i_force_y, k_i_force_z,
    //   k_i_moment_x, k_i_moment_y, k_i_moment_z.
    //
    // In other words, this node assumes the plant dynamics model is already known
    // and that only the external wrench is unknown.
    k_i_force_.x() = this->declare_parameter<double>("k_i_force_x", kDefaultKiForceX);
    k_i_force_.y() = this->declare_parameter<double>("k_i_force_y", kDefaultKiForceY);
    k_i_force_.z() = this->declare_parameter<double>("k_i_force_z", kDefaultKiForceZ);
    k_i_moment_.x() = this->declare_parameter<double>("k_i_moment_x", kDefaultKiMomentX);
    k_i_moment_.y() = this->declare_parameter<double>("k_i_moment_y", kDefaultKiMomentY);
    k_i_moment_.z() = this->declare_parameter<double>("k_i_moment_z", kDefaultKiMomentZ);

    mass_ = this->declare_parameter<double>("mass", 4.4);
    gravity_ = this->declare_parameter<double>("gravity", 9.81);
    inertia_xx_ = this->declare_parameter<double>("inertia_xx", 0.360702);
    inertia_yy_ = this->declare_parameter<double>("inertia_yy", 0.360702);
    inertia_zz_ = this->declare_parameter<double>("inertia_zz", 0.660702);
    thrust_coefficient_ = this->declare_parameter<double>("thrust_coefficient", 0.02);
    reaction_torque_coefficient_ = this->declare_parameter<double>("reaction_torque_coefficient", 0.02);
    arm_radius_ = this->declare_parameter<double>("arm_radius", 0.148492);
    rotor_z_offset_ = this->declare_parameter<double>("rotor_z_offset", 0.075);
    com_x_ = this->declare_parameter<double>("com_x", 0.0);
    com_y_ = this->declare_parameter<double>("com_y", 0.0);
    com_z_ = this->declare_parameter<double>("com_z", 0.0);
    state_dt_ = this->declare_parameter<double>("state_dt", 1.0 / 400.0);

    inertia_body_.setZero();
    inertia_body_(0, 0) = inertia_xx_;
    inertia_body_(1, 1) = inertia_yy_;
    inertia_body_(2, 2) = inertia_zz_;

    sub_input_ = this->create_subscription<palletrone_interfaces::msg::Input>(
      "/input", 10, std::bind(&WrenchObserver::onInput, this, std::placeholders::_1));
    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>(
      "/palletrone_state", 10, std::bind(&WrenchObserver::onState, this, std::placeholders::_1));

    pub_external_wrench_hat = this->create_publisher<palletrone_interfaces::msg::Wrench>("/external_wrench_hat", 10);
    pub_external_wrench_raw_ = this->create_publisher<palletrone_interfaces::msg::Wrench>("/external_wrench_raw", 10);

  }

private:
  void onInput(const palletrone_interfaces::msg::Input::SharedPtr msg)
  {
    for (size_t i = 0; i < 8; ++i) {
      input_u_[i] = static_cast<double>(msg->u[i]);
    }
    have_input_ = true;
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
  {
    // State-estimation assumption:
    // This node does not run an EKF, UKF, or any other state observer internally.
    // It assumes the plant already provides usable state estimates through
    // /palletrone_state, and it consumes them directly.
    //
    // Practically, the observer receives:
    // - v_W from msg->vel      : linear velocity in world frame W
    // - RPY from msg->rpy      : attitude of body B relative to world W
    // - omega_B from msg->w_rpy: angular velocity in body frame B
    // - measured servo angles  : msg->servo, converted from deg to rad
    //
    // Therefore, if an external EKF/state estimator exists in the overall system,
    // this node is not connected to it explicitly. It simply trusts the contents of
    // /palletrone_state as the available state estimate.

    if (!have_input_) {
      have_prev_state_ = false;
      return;
    }

    // State message conventions used here:
    // - vel: linear velocity expressed in world frame W, v_W
    // - rpy: body attitude relative to world frame W
    // - w_rpy: body angular velocity expressed in body frame B, omega_B
    Eigen::Vector3d vel_world(
      static_cast<double>(msg->vel[0]),
      static_cast<double>(msg->vel[1]),
      static_cast<double>(msg->vel[2]));
    Eigen::Vector3d rpy(
      static_cast<double>(msg->rpy[0]),
      static_cast<double>(msg->rpy[1]),
      static_cast<double>(msg->rpy[2]));
    Eigen::Vector3d omega_body(
      static_cast<double>(msg->w_rpy[0]),
      static_cast<double>(msg->w_rpy[1]),
      static_cast<double>(msg->w_rpy[2]));

    std::array<double, 4> servo_rad{};
    for (size_t i = 0; i < 4; ++i) {
      servo_rad[i] = static_cast<double>(msg->servo[i]) * kDegToRad;
    }

    // R_WB maps a vector from body frame B to world frame W:
    //   v_W = R_WB * v_B
    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy);

    // 6-DoF momentum definition:
    //   p_W = m * v_W                [linear momentum in world frame]
    //   h_B = I_B * omega_B         [angular momentum in body frame]
    const Eigen::Vector3d linear_momentum = mass_ * vel_world;
    const Eigen::Vector3d angular_momentum = inertia_body_ * omega_body;

    if (!have_prev_state_) {
      prev_linear_momentum_ = linear_momentum;
      prev_angular_momentum_ = angular_momentum;
      prev_external_force_body_.setZero();
      prev_external_moment_body_.setZero();
      have_prev_state_ = true;
      return;
    }

    // The plant publishes /palletrone_state using MuJoCo simulation time at a fixed
    // physics/update rate. This observer does not receive that simulation timestamp,
    // so using wall-clock time here introduces a scale error whenever ROS callback
    // timing deviates from the simulator's internal time progression.
    //
    // To stay aligned with the plant-side dynamics, use the known state publication
    // period instead of this->now():
    //   dt = 1 / state_rate
    //
    // Default:
    //   state_dt = 1/400 s
    // matching PHYSICS_HZ in plant.py.
    const double dt = state_dt_;

    // Numerical differentiation of momentum:
    //   p_dot_W ~= (p_W[k] - p_W[k-1]) / dt
    //   h_dot_B ~= (h_B[k] - h_B[k-1]) / dt
    //
    // So even without an EKF inside this node, the observer still estimates
    // momentum derivatives from the incoming state stream by finite difference.
    const Eigen::Vector3d linear_momentum_dot = (linear_momentum - prev_linear_momentum_) / dt;
    const Eigen::Vector3d angular_momentum_dot = (angular_momentum - prev_angular_momentum_) / dt;
    prev_linear_momentum_ = linear_momentum;
    prev_angular_momentum_ = angular_momentum;

    const ActuationWrench actuation = computeActuationWrench(servo_rad);

    // Gravity in world frame:
    //   g_W = [0, 0, -m g]^T
    const Eigen::Vector3d gravity_world(0.0, 0.0, -mass_ * gravity_);

    // Translational rigid-body dynamics:
    //   p_dot_W = R_WB * F_act_B + F_ext_W + g_W
    // Rearranged for the external wrench estimate:
    //   F_ext_W = p_dot_W - R_WB * F_act_B - g_W
    const Eigen::Vector3d raw_force_world =
      linear_momentum_dot - r_wb * actuation.force_body - gravity_world;

    // The observer publishes force in body frame B, so convert:
    //   F_ext_B = R_BW * F_ext_W = R_WB^T * F_ext_W
    const Eigen::Vector3d raw_force_body = r_wb.transpose() * raw_force_world;

    // Rotational rigid-body dynamics in body frame:
    //   h_dot_B + omega_B x h_B = tau_act_B + tau_ext_B
    // Rearranged for the external moment estimate:
    //   tau_ext_B = h_dot_B + omega_B x h_B - tau_act_B
    const Eigen::Vector3d raw_moment_body =
      angular_momentum_dot + omega_body.cross(angular_momentum) - actuation.moment_body;

    // Diagonal momentum-observer gain with exact first-order discretization:
    //   alpha = 1 - exp(-K_I * dt)
    // This keeps K_I as the observer bandwidth [1/s] without adding a separate LPF.
    const Eigen::Vector3d alpha_force = observerAlpha(k_i_force_, dt);
    const Eigen::Vector3d alpha_moment = observerAlpha(k_i_moment_, dt);
    prev_external_force_body_ += alpha_force.cwiseProduct(raw_force_body - prev_external_force_body_);
    prev_external_moment_body_ += alpha_moment.cwiseProduct(raw_moment_body - prev_external_moment_body_);

    publishWrench(pub_external_wrench_raw_, raw_force_body, raw_moment_body);
    publishWrench(pub_external_wrench_hat, prev_external_force_body_, prev_external_moment_body_);
  }

  struct ActuationWrench
  {
    Eigen::Vector3d force_body{Eigen::Vector3d::Zero()};
    Eigen::Vector3d moment_body{Eigen::Vector3d::Zero()};
  };

  Eigen::Matrix3d rotationWorldFromBody(const Eigen::Vector3d & rpy) const
  {
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
    // ZYX convention, body frame B to world frame W:
    //   R_WB = Rz(yaw) * Ry(pitch) * Rx(roll)
    r_wb <<
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr;
    return r_wb;
  }

  Eigen::Vector3d observerAlpha(const Eigen::Vector3d & gain, double dt) const
  {
    Eigen::Vector3d alpha;
    for (int i = 0; i < 3; ++i) {
      alpha(i) = 1.0 - std::exp(-std::max(0.0, gain(i)) * std::max(0.0, dt));
    }
    return alpha;
  }

  Eigen::Matrix4d calcA1(const Eigen::Vector4d & servo_angle) const
  {
    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);

    const double s1 = std::sin(servo_angle(0));
    const double s2 = std::sin(servo_angle(1));
    const double s3 = std::sin(servo_angle(2));
    const double s4 = std::sin(servo_angle(3));
    const double c1 = std::cos(servo_angle(0));
    const double c2 = std::cos(servo_angle(1));
    const double c3 = std::cos(servo_angle(2));
    const double c4 = std::cos(servo_angle(3));

    Eigen::Matrix4d a1;
    // A1 maps rotor thrust magnitudes C1 = [f1 f2 f3 f4]^T to
    // body-frame roll/pitch/yaw moments and collective body-z force:
    //
    //   B1 = A1(C2) * C1
    //   B1 = [Mx_B, My_B, Mz_reaction_B, Fz_B]^T
    //
    // where C2 are servo tilt angles in body frame B.
    a1(0, 0) = inv_sqrt2 * (reaction_torque_coefficient_ + rotor_z_offset_ - com_z_) * s1 +
      (+arm_radius_ - com_y_) * c1;
    a1(0, 1) = inv_sqrt2 * (-reaction_torque_coefficient_ - rotor_z_offset_ + com_z_) * s2 +
      (+arm_radius_ - com_y_) * c2;
    a1(0, 2) = inv_sqrt2 * (-reaction_torque_coefficient_ - rotor_z_offset_ + com_z_) * s3 +
      (-arm_radius_ - com_y_) * c3;
    a1(0, 3) = inv_sqrt2 * (reaction_torque_coefficient_ + rotor_z_offset_ - com_z_) * s4 +
      (-arm_radius_ - com_y_) * c4;

    a1(1, 0) = inv_sqrt2 * (-reaction_torque_coefficient_ + rotor_z_offset_ - com_z_) * s1 +
      (-arm_radius_ + com_x_) * c1;
    a1(1, 1) = inv_sqrt2 * (-reaction_torque_coefficient_ + rotor_z_offset_ - com_z_) * s2 +
      (+arm_radius_ + com_x_) * c2;
    a1(1, 2) = inv_sqrt2 * (reaction_torque_coefficient_ - rotor_z_offset_ + com_z_) * s3 +
      (+arm_radius_ + com_x_) * c3;
    a1(1, 3) = inv_sqrt2 * (reaction_torque_coefficient_ - rotor_z_offset_ + com_z_) * s4 +
      (-arm_radius_ + com_x_) * c4;

    a1(2, 0) = reaction_torque_coefficient_ * c1;
    a1(2, 1) = -reaction_torque_coefficient_ * c2;
    a1(2, 2) = reaction_torque_coefficient_ * c3;
    a1(2, 3) = -reaction_torque_coefficient_ * c4;

    a1(3, 0) = c1;
    a1(3, 1) = c2;
    a1(3, 2) = c3;
    a1(3, 3) = c4;

    return a1;
  }

  Eigen::Matrix4d calcA2(const Eigen::Vector4d & thrust, const Eigen::Vector4d & servo_angle) const
  {
    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    const double l_arm = std::sqrt(2.0) * arm_radius_;

    const double s1 = std::sin(servo_angle(0));
    const double s2 = std::sin(servo_angle(1));
    const double s3 = std::sin(servo_angle(2));
    const double s4 = std::sin(servo_angle(3));

    const double f1 = thrust(0);
    const double f2 = thrust(1);
    const double f3 = thrust(2);
    const double f4 = thrust(3);

    Eigen::Matrix4d a2;
    // A2 maps tilt-direction sine terms S = [sin(th1) ... sin(th4)]^T to
    // body-frame lateral force and additional yaw moment terms:
    //
    //   B2 = A2(C1, C2) * S
    //   B2 = [Fx_B, Fy_B, Mz_tilt_B, 0]^T
    //
    // Combined actuation wrench used by this observer:
    //   F_act_B   = [Fx_B, Fy_B, Fz_B]^T
    //   tau_act_B = [Mx_B, My_B, Mz_reaction_B + Mz_tilt_B]^T
    a2(0, 0) = inv_sqrt2 * f1;
    a2(0, 1) = inv_sqrt2 * f2;
    a2(0, 2) = -inv_sqrt2 * f3;
    a2(0, 3) = -inv_sqrt2 * f4;

    a2(1, 0) = -inv_sqrt2 * f1;
    a2(1, 1) = inv_sqrt2 * f2;
    a2(1, 2) = inv_sqrt2 * f3;
    a2(1, 3) = -inv_sqrt2 * f4;

    a2(2, 0) = inv_sqrt2 * (+com_x_ + com_y_) * s1 - l_arm * f1;
    a2(2, 1) = inv_sqrt2 * (-com_x_ + com_y_) * s2 - l_arm * f2;
    a2(2, 2) = inv_sqrt2 * (-com_x_ - com_y_) * s3 - l_arm * f3;
    a2(2, 3) = inv_sqrt2 * (+com_x_ - com_y_) * s4 - l_arm * f4;

    a2(3, 0) = -2.0 * l_arm * f1;
    a2(3, 1) = 2.0 * l_arm * f2;
    a2(3, 2) = -2.0 * l_arm * f3;
    a2(3, 3) = 2.0 * l_arm * f4;

    return a2;
  }

  ActuationWrench computeActuationWrench(const std::array<double, 4> & servo_rad) const
  {
    Eigen::Vector4d servo;
    Eigen::Vector4d thrust;
    Eigen::Vector4d sin_servo;
    for (size_t i = 0; i < 4; ++i) {
      // Input message convention:
      //   u[0:4] = motor angular speeds
      //   u[4:8] = commanded servo angles
      //
      // The observer intentionally uses measured servo angles from /palletrone_state
      // rather than commanded servo angles, because the observer should reflect the
      // actual plant-side actuation applied to the body.
      //
      // This means actuation is reconstructed from:
      // - /input               : commanded motor angular speeds
      // - /palletrone_state    : measured servo angles
      //
      // No separate actuator-state estimator is used here.
      servo(i) = servo_rad[i];
      sin_servo(i) = std::sin(servo_rad[i]);

      // MuJoCo actuator model used by the plant:
      //   f_i = k_thrust * omega_i^2
      const double motor_speed = std::max(0.0, input_u_[i]);
      thrust(i) = thrust_coefficient_ * motor_speed * motor_speed;
    }

    // Reconstruct the body-frame actuation wrench from the same allocation model
    // used by allocator_controller.
    const Eigen::Vector4d b1 = calcA1(servo) * thrust;
    const Eigen::Vector4d b2 = calcA2(thrust, servo) * sin_servo;

    ActuationWrench out;
    // All outputs here are expressed in body frame B.
    out.force_body << b2(0), b2(1), b1(3); // B1 = [Mx_B, My_B, Mz_reaction_B, Fz_B]^T
    //
    out.moment_body << b1(0), b1(1), b1(2) + b2(2); //B2 = [Fx_B, Fy_B, Mz_tilt_B, 0]^T
    return out;
  }

  void publishWrench(
    const rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr & publisher,
    const Eigen::Vector3d & force_body,
    const Eigen::Vector3d & moment_body) const
  {
    palletrone_interfaces::msg::Wrench msg;
    msg.force[0] = static_cast<float>(force_body.x());
    msg.force[1] = static_cast<float>(force_body.y());
    msg.force[2] = static_cast<float>(force_body.z());
    msg.moment[0] = static_cast<float>(moment_body.x());
    msg.moment[1] = static_cast<float>(moment_body.y());
    msg.moment[2] = static_cast<float>(moment_body.z());
    publisher->publish(msg);
  }

  rclcpp::Subscription<palletrone_interfaces::msg::Input>::SharedPtr sub_input_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_external_wrench_hat;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_external_wrench_raw_;

  double mass_{4.4};
  double gravity_{9.81};
  double inertia_xx_{0.360702};
  double inertia_yy_{0.360702};
  double inertia_zz_{0.660702};
  double thrust_coefficient_{0.02};
  double reaction_torque_coefficient_{0.02};
  double arm_radius_{0.148492};
  double rotor_z_offset_{0.075};
  double com_x_{0.0};
  double com_y_{0.0};
  double com_z_{0.0};
  Eigen::Vector3d k_i_force_{
    kDefaultKiForceX,
    kDefaultKiForceY,
    kDefaultKiForceZ};
  Eigen::Vector3d k_i_moment_{
    kDefaultKiMomentX,
    kDefaultKiMomentY,
    kDefaultKiMomentZ};
  double state_dt_{1.0 / 400.0};

  Eigen::Matrix3d inertia_body_{Eigen::Matrix3d::Zero()};
  std::array<double, 8> input_u_{};
  bool have_input_{false};
  bool have_prev_state_{false};
  Eigen::Vector3d prev_linear_momentum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_angular_momentum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_external_force_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_external_moment_body_{Eigen::Vector3d::Zero()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchObserver>());
  rclcpp::shutdown();
  return 0;
}
