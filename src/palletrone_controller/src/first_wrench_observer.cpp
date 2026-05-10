#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/mob_observer_input.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

class WrenchObserver : public rclcpp::Node
{
public:
  static constexpr double kDefaultKForceX = 5.0;
  static constexpr double kDefaultKForceY = 5.0;
  static constexpr double kDefaultKForceZ = 5.0;
  static constexpr double kDefaultKMomentX = 10.0;
  static constexpr double kDefaultKMomentY = 10.0;
  static constexpr double kDefaultKMomentZ = 10.0;

  WrenchObserver()
  : rclcpp::Node("first_wrench_observer")
  {
    k_force_.x() = this->declare_parameter<double>("k_i_force_x", kDefaultKForceX);
    k_force_.y() = this->declare_parameter<double>("k_i_force_y", kDefaultKForceY);
    k_force_.z() = this->declare_parameter<double>("k_i_force_z", kDefaultKForceZ);
    k_moment_.x() = this->declare_parameter<double>("k_i_moment_x", kDefaultKMomentX);
    k_moment_.y() = this->declare_parameter<double>("k_i_moment_y", kDefaultKMomentY);
    k_moment_.z() = this->declare_parameter<double>("k_i_moment_z", kDefaultKMomentZ);

    mass_ = this->declare_parameter<double>("mass", 4.0);
    gravity_ = this->declare_parameter<double>("gravity", 9.81);
    inertia_xx_ = this->declare_parameter<double>("inertia_xx", 0.360702);
    inertia_yy_ = this->declare_parameter<double>("inertia_yy", 0.360702);
    inertia_zz_ = this->declare_parameter<double>("inertia_zz", 0.660702);
    state_dt_ = this->declare_parameter<double>("state_dt", 1.0 / 400.0);

    inertia_body_.setZero();
    inertia_body_(0, 0) = inertia_xx_;
    inertia_body_(1, 1) = inertia_yy_;
    inertia_body_(2, 2) = inertia_zz_;

    sub_mob_observer_input_ =
      this->create_subscription<palletrone_interfaces::msg::MobObserverInput>(
      "/mob_observer_input", 10,
      std::bind(&WrenchObserver::onMobObserverInput, this, std::placeholders::_1));

    pub_external_wrench_hat_ = this->create_publisher<palletrone_interfaces::msg::Wrench>(
      "/external_wrench_hat", 10);
  }

private:
  void onMobObserverInput(const palletrone_interfaces::msg::MobObserverInput::SharedPtr msg)
  {
    // /mob_observer_input contains state and actuation wrench from the same mj_step().
    // This avoids pairing state(k) with actuation_wrench(k-1) through two independent topics.
    const Eigen::Vector3d vel_world(
      static_cast<double>(msg->vel[0]),
      static_cast<double>(msg->vel[1]),
      static_cast<double>(msg->vel[2]));
    const Eigen::Vector3d rpy(
      static_cast<double>(msg->rpy[0]),
      static_cast<double>(msg->rpy[1]),
      static_cast<double>(msg->rpy[2]));
    const Eigen::Vector3d omega_body(
      static_cast<double>(msg->w_rpy[0]),
      static_cast<double>(msg->w_rpy[1]),
      static_cast<double>(msg->w_rpy[2]));
    const Eigen::Vector3d actuation_force_body(
      static_cast<double>(msg->actuation_force[0]),
      static_cast<double>(msg->actuation_force[1]),
      static_cast<double>(msg->actuation_force[2]));
    const Eigen::Vector3d actuation_moment_body(
      static_cast<double>(msg->actuation_moment[0]),
      static_cast<double>(msg->actuation_moment[1]),
      static_cast<double>(msg->actuation_moment[2]));

    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy);

    // Momentum states used by the first-order MoB:
    //   p_W = m v_W
    //   h_B = J_B omega_B
    const Eigen::Vector3d linear_momentum_world = mass_ * vel_world;
    const Eigen::Vector3d angular_momentum_body = inertia_body_ * omega_body;

    if (!have_observer_state_) {
      // Initialize z with the current momentum so the observer starts from zero
      // estimated disturbance instead of producing a startup impulse.
      z_f_world_ = linear_momentum_world;
      z_tau_body_ = angular_momentum_body;
      external_force_hat_world_.setZero();
      external_moment_hat_body_.setZero();
      have_observer_state_ = true;
      prev_sim_time_ = static_cast<double>(msg->sim_time);
      have_prev_time_ = true;
      return;
    }

    double dt = state_dt_;
    if (have_prev_time_) {
      dt = static_cast<double>(msg->sim_time) - prev_sim_time_;
      dt = std::clamp(dt, 1e-6, 0.05);
    }
    prev_sim_time_ = static_cast<double>(msg->sim_time);
    have_prev_time_ = true;

    // Known input terms in the first-order MoB:
    //   u_f_W   = R_WB f_act_B + mg_W
    //   u_tau_B = tau_act_B - omega_B x h_B
    const Eigen::Vector3d gravity_world(0.0, 0.0, -mass_ * gravity_);
    const Eigen::Vector3d u_f_world =
      r_wb * actuation_force_body + gravity_world;
    const Eigen::Vector3d u_tau_body =
      actuation_moment_body - omega_body.cross(angular_momentum_body);

    // First-order momentum observer output:
    //   f_hat_ext_W   = K_f   (p_W - z_f)
    //   tau_hat_ext_B = K_tau (h_B - z_tau)
    external_force_hat_world_ =
      k_force_.cwiseProduct(linear_momentum_world - z_f_world_);
    external_moment_hat_body_ =
      k_moment_.cwiseProduct(angular_momentum_body - z_tau_body_);

    // Observer internal state integration:
    //   z_dot_f   = u_f_W + f_hat_ext_W
    //   z_dot_tau = u_tau_B + tau_hat_ext_B
    // No finite difference of p_W or h_B is used.
    z_f_world_ += dt * (u_f_world + external_force_hat_world_);
    z_tau_body_ += dt * (u_tau_body + external_moment_hat_body_);

    // The force observer runs in world frame, but wrench_controller currently consumes
    // body-frame force/moment on /external_wrench_hat.
    const Eigen::Vector3d external_force_hat_body = r_wb.transpose() * external_force_hat_world_;
    publishWrench(pub_external_wrench_hat_, external_force_hat_body, external_moment_hat_body_);
  }

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
    r_wb <<
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr;
    return r_wb;
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

  rclcpp::Subscription<palletrone_interfaces::msg::MobObserverInput>::SharedPtr
    sub_mob_observer_input_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_external_wrench_hat_;

  double mass_{4.0};
  double gravity_{9.81};
  double inertia_xx_{0.360702};
  double inertia_yy_{0.360702};
  double inertia_zz_{0.660702};
  double state_dt_{1.0 / 400.0};
  double prev_sim_time_{0.0};

  Eigen::Vector3d k_force_{kDefaultKForceX, kDefaultKForceY, kDefaultKForceZ};
  Eigen::Vector3d k_moment_{kDefaultKMomentX, kDefaultKMomentY, kDefaultKMomentZ};
  Eigen::Matrix3d inertia_body_{Eigen::Matrix3d::Zero()};

  Eigen::Vector3d z_f_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d z_tau_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_hat_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_moment_hat_body_{Eigen::Vector3d::Zero()};

  bool have_observer_state_{false};
  bool have_prev_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchObserver>());
  rclcpp::shutdown();
  return 0;
}
