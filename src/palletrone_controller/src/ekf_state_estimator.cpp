#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace
{
constexpr int kStateDim = 15;
constexpr int kMeasDim = 9;
constexpr double kDefaultDt = 1.0 / 400.0;
constexpr double kGravity = 9.81;

using VectorState = Eigen::Matrix<double, kStateDim, 1>;
using MatrixState = Eigen::Matrix<double, kStateDim, kStateDim>;
using VectorMeas = Eigen::Matrix<double, kMeasDim, 1>;
using MatrixMeas = Eigen::Matrix<double, kMeasDim, kMeasDim>;
using MatrixHx = Eigen::Matrix<double, kMeasDim, kStateDim>;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d wrapAngles(const Eigen::Vector3d & angles)
{
  return Eigen::Vector3d(wrapAngle(angles.x()), wrapAngle(angles.y()), wrapAngle(angles.z()));
}

Eigen::Vector3d arrayToVector3(const std::array<double, 3> & data)
{
  return Eigen::Vector3d(data[0], data[1], data[2]);
}

Eigen::Matrix3d rotationWorldFromBody(const Eigen::Vector3d & rpy)
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
  r_wb << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
              -sp,                     cp * sr,                     cp * cr;
  return r_wb;
}

Eigen::Matrix3d eulerRateMatrixBodyToRpyRate(const Eigen::Vector3d & rpy)
{
  const double roll = rpy.x();
  const double pitch = std::clamp(rpy.y(), -1.55, 1.55);
  const double sr = std::sin(roll);
  const double cr = std::cos(roll);
  const double tp = std::tan(pitch);
  const double cp = std::cos(pitch);

  Eigen::Matrix3d t;
  t << 1.0, sr * tp, cr * tp,
       0.0,      cr,     -sr,
       0.0, sr / cp, cr / cp;
  return t;
}
}  // namespace

class EkfStateEstimator : public rclcpp::Node
{
public:
  EkfStateEstimator()
  : rclcpp::Node("ekf_state_estimator")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/palletrone_state");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/palletrone_state_ekf");
    fixed_dt_ = this->declare_parameter<double>("fixed_dt", kDefaultDt);
    use_ros_time_dt_ = this->declare_parameter<bool>("use_ros_time_dt", false);

    // msg.acc는 plant에서 body 좌표계 IMU specific force로 만든 값이다.
    // 따라서 EKF 예측에서는 f_B를 R_WB로 world frame에 회전시킨 뒤 중력을 다시 더한다.
    //   a_W = R_WB * (f_B - b_a) + g_W,  g_W = [0, 0, -9.81]
    pos_meas_std_ = this->declare_parameter<double>("pos_meas_std", 1.0e-3);
    vel_meas_std_ = this->declare_parameter<double>("vel_meas_std", 1.0e-3);
    rpy_meas_std_ = this->declare_parameter<double>("rpy_meas_std", 2.0e-3);
    accel_noise_std_ = this->declare_parameter<double>("accel_noise_std", 5.0e-2);
    gyro_noise_std_ = this->declare_parameter<double>("gyro_noise_std", 5.0e-3);
    accel_bias_walk_std_ = this->declare_parameter<double>("accel_bias_walk_std", 1.0e-3);
    gyro_bias_walk_std_ = this->declare_parameter<double>("gyro_bias_walk_std", 1.0e-4);

    initialiseMatrices();

    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>(
      input_topic_, 10, std::bind(&EkfStateEstimator::onState, this, std::placeholders::_1));
    pub_state_ = this->create_publisher<palletrone_interfaces::msg::PalletroneState>(
      output_topic_, 10);
    pub_state_vector_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ekf/state_vector", 10);
    pub_bias_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ekf/bias", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "EKF state estimator started: input=%s, output=%s, fixed_dt=%.6f",
      input_topic_.c_str(), output_topic_.c_str(), fixed_dt_);
  }

private:
  void initialiseMatrices()
  {
    x_.setZero();
    P_.setIdentity();

    // 초기 공분산은 상태마다 다르게 둔다. bias는 천천히 학습되도록 조금 크게 시작한다.
    P_.block<3, 3>(0, 0) *= 1.0e-3;    // 위치
    P_.block<3, 3>(3, 3) *= 1.0e-3;    // 속도
    P_.block<3, 3>(6, 6) *= 1.0e-3;    // 자세
    P_.block<3, 3>(9, 9) *= 1.0e-2;    // 가속도 bias
    P_.block<3, 3>(12, 12) *= 1.0e-3;  // 자이로 bias

    H_.setZero();
    H_.block<3, 3>(0, 0).setIdentity();  // 위치 관측
    H_.block<3, 3>(3, 3).setIdentity();  // 속도 관측
    H_.block<3, 3>(6, 6).setIdentity();  // RPY 관측
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
  {
    const Eigen::Vector3d pos_meas = arrayToVector3(msg->pos);
    const Eigen::Vector3d vel_meas = arrayToVector3(msg->vel);
    const Eigen::Vector3d specific_force_body = arrayToVector3(msg->acc);
    const Eigen::Vector3d rpy_meas = wrapAngles(arrayToVector3(msg->rpy));
    const Eigen::Vector3d gyro_body = arrayToVector3(msg->w_rpy);

    if (!initialised_) {
      // 첫 샘플은 PX4 EKF의 alignment 단계처럼 측정 상태로 초기화한다.
      x_.segment<3>(0) = pos_meas;
      x_.segment<3>(3) = vel_meas;
      x_.segment<3>(6) = rpy_meas;
      x_.segment<3>(9).setZero();
      x_.segment<3>(12).setZero();
      last_time_ = this->now();
      initialised_ = true;
      publishState(*msg);
      return;
    }

    const double dt = computeDt();
    predict(specific_force_body, gyro_body, dt);
    fusePositionVelocityAttitude(pos_meas, vel_meas, rpy_meas);
    publishState(*msg);
  }

  double computeDt()
  {
    if (!use_ros_time_dt_) {
      return sanitiseDt(fixed_dt_);
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    return sanitiseDt(dt);
  }

  double sanitiseDt(double dt) const
  {
    if (!(dt > 0.0) || dt > 0.05) {
      return kDefaultDt;
    }
    return dt;
  }

  void predict(
    const Eigen::Vector3d & specific_force_body,
    const Eigen::Vector3d & gyro_body,
    double dt)
  {
    const Eigen::Vector3d pos = x_.segment<3>(0);
    const Eigen::Vector3d vel = x_.segment<3>(3);
    const Eigen::Vector3d rpy = x_.segment<3>(6);
    const Eigen::Vector3d accel_bias = x_.segment<3>(9);
    const Eigen::Vector3d gyro_bias = x_.segment<3>(12);

    // 물리 모델:
    //   f_B = accelerometer specific force, body frame
    //   b_a = accelerometer bias, body frame
    //   R_WB = body frame 벡터를 world frame으로 옮기는 회전행렬
    //   g_W = world frame 중력벡터 [0, 0, -g]
    //
    // 가속도계는 선가속도 a_W가 아니라 specific force f_B를 측정하므로,
    // world 선가속도는 아래처럼 복원한다.
    //   a_W = R_WB * (f_B - b_a) + g_W
    //
    // 자이로는 body angular velocity [p, q, r]를 측정하고, 이는 RPY 미분과 같지 않다.
    // 그래서 Euler rate 변환행렬 T(r,p)을 사용한다.
    //   rpy_dot = T(r,p) * (w_B - b_g)
    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy);
    const Eigen::Matrix3d t_rpy = eulerRateMatrixBodyToRpyRate(rpy);
    const Eigen::Vector3d gravity_world(0.0, 0.0, -kGravity);
    const Eigen::Vector3d acc_world = r_wb * (specific_force_body - accel_bias) + gravity_world;
    const Eigen::Vector3d rpy_rate = t_rpy * (gyro_body - gyro_bias);

    x_.segment<3>(0) = pos + vel * dt + 0.5 * acc_world * dt * dt;
    x_.segment<3>(3) = vel + acc_world * dt;
    x_.segment<3>(6) = wrapAngles(rpy + rpy_rate * dt);

    // bias는 random walk로 두고, 값이 비현실적으로 커지지 않게 완만히 제한한다.
    x_.segment<3>(9) = x_.segment<3>(9).cwiseMax(-5.0).cwiseMin(5.0);
    x_.segment<3>(12) = x_.segment<3>(12).cwiseMax(-1.0).cwiseMin(1.0);

    MatrixState F = MatrixState::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(0, 9) = -0.5 * r_wb * dt * dt;
    F.block<3, 3>(3, 9) = -r_wb * dt;
    F.block<3, 3>(6, 12) = -t_rpy * dt;

    MatrixState Q = MatrixState::Zero();
    const double accel_q = accel_noise_std_ * accel_noise_std_;
    const double gyro_q = gyro_noise_std_ * gyro_noise_std_;
    const double accel_bias_q = accel_bias_walk_std_ * accel_bias_walk_std_;
    const double gyro_bias_q = gyro_bias_walk_std_ * gyro_bias_walk_std_;

    // 연속 white noise를 단순 이산화한 값이다. MuJoCo 400 Hz 기준에서 충분히 안정적이다.
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.25 * dt * dt * dt * dt * accel_q);
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (dt * dt * accel_q);
    Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (dt * dt * gyro_q);
    Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (dt * accel_bias_q);
    Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (dt * gyro_bias_q);

    P_ = F * P_ * F.transpose() + Q;
    P_ = 0.5 * (P_ + P_.transpose());
  }

  void fusePositionVelocityAttitude(
    const Eigen::Vector3d & pos_meas,
    const Eigen::Vector3d & vel_meas,
    const Eigen::Vector3d & rpy_meas)
  {
    VectorMeas z;
    z.segment<3>(0) = pos_meas;
    z.segment<3>(3) = vel_meas;
    z.segment<3>(6) = rpy_meas;

    VectorMeas h;
    h.segment<3>(0) = x_.segment<3>(0);
    h.segment<3>(3) = x_.segment<3>(3);
    h.segment<3>(6) = x_.segment<3>(6);

    VectorMeas innov = z - h;
    innov.segment<3>(6) = wrapAngles(innov.segment<3>(6));

    MatrixMeas R = MatrixMeas::Zero();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * pos_meas_std_ * pos_meas_std_;
    R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * vel_meas_std_ * vel_meas_std_;
    R.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * rpy_meas_std_ * rpy_meas_std_;

    const MatrixMeas S = H_ * P_ * H_.transpose() + R;
    const Eigen::Matrix<double, kStateDim, kMeasDim> K =
      P_ * H_.transpose() * S.ldlt().solve(MatrixMeas::Identity());

    x_ += K * innov;
    x_.segment<3>(6) = wrapAngles(x_.segment<3>(6));

    // Joseph form은 수치 오차로 P가 비양정이 되는 것을 줄여준다.
    const MatrixState I = MatrixState::Identity();
    const MatrixState I_KH = I - K * H_;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
  }

  void publishState(const palletrone_interfaces::msg::PalletroneState & input_msg)
  {
    palletrone_interfaces::msg::PalletroneState out;

    for (size_t i = 0; i < 3; ++i) {
      out.pos[i] = x_(static_cast<int>(i));
      out.vel[i] = x_(3 + static_cast<int>(i));
      out.rpy[i] = x_(6 + static_cast<int>(i));
      out.acc[i] = input_msg.acc[i];
      out.w_rpy[i] = input_msg.w_rpy[i] - x_(12 + static_cast<int>(i));
      out.a_rpy[i] = input_msg.a_rpy[i];
    }

    // servo는 상태추정 대상이 아니라 actuator 측정값이므로 그대로 통과시킨다.
    out.servo = input_msg.servo;

    pub_state_->publish(out);
    publishDebugTopics();
  }

  void publishDebugTopics()
  {
    // ros2 topic echo로 EKF 내부 상태를 바로 확인하기 위한 debug 토픽이다.
    // state_vector 순서:
    //   [pos_x, pos_y, pos_z,
    //    vel_x, vel_y, vel_z,
    //    roll, pitch, yaw,
    //    accel_bias_x, accel_bias_y, accel_bias_z,
    //    gyro_bias_x, gyro_bias_y, gyro_bias_z]
    std_msgs::msg::Float64MultiArray state_vector_msg;
    state_vector_msg.data.resize(kStateDim);
    for (int i = 0; i < kStateDim; ++i) {
      state_vector_msg.data[static_cast<size_t>(i)] = x_(i);
    }
    pub_state_vector_->publish(state_vector_msg);

    // bias 순서:
    //   [accel_bias_x, accel_bias_y, accel_bias_z,
    //    gyro_bias_x, gyro_bias_y, gyro_bias_z]
    std_msgs::msg::Float64MultiArray bias_msg;
    bias_msg.data.resize(6);
    for (int i = 0; i < 3; ++i) {
      bias_msg.data[static_cast<size_t>(i)] = x_(9 + i);
      bias_msg.data[static_cast<size_t>(3 + i)] = x_(12 + i);
    }
    pub_bias_->publish(bias_msg);
  }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::PalletroneState>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_vector_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_bias_;

  VectorState x_{VectorState::Zero()};
  MatrixState P_{MatrixState::Identity()};
  MatrixHx H_{MatrixHx::Zero()};

  rclcpp::Time last_time_;
  bool initialised_{false};

  double fixed_dt_{kDefaultDt};
  bool use_ros_time_dt_{false};

  double pos_meas_std_{1.0e-3};
  double vel_meas_std_{1.0e-3};
  double rpy_meas_std_{2.0e-3};
  double accel_noise_std_{5.0e-2};
  double gyro_noise_std_{5.0e-3};
  double accel_bias_walk_std_{1.0e-3};
  double gyro_bias_walk_std_{1.0e-4};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfStateEstimator>());
  rclcpp::shutdown();
  return 0;
}
