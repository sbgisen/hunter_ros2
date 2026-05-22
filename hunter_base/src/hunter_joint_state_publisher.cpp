/*
 * joint_state_publisher_node.cpp
 */

#include "hunter_base/joint_state_publisher.hpp"

#include <cmath>
#include <memory>

#include "hunter_base/hunter_params.hpp"

namespace westonrobot {

JointStatePublisher::JointStatePublisher()
    : rclcpp::Node("joint_state_publisher") {
  declare_parameter<std::string>("robot_model", "hunter_se");
  declare_parameter<std::string>("status_topic", "/hunter_status");
  declare_parameter<std::string>("joint_states_topic", "/joint_states");
  declare_parameter<std::string>("fl_wheel_joint", "fl_wheel_joint");
  declare_parameter<std::string>("fr_wheel_joint", "fr_wheel_joint");
  declare_parameter<std::string>("bl_wheel_joint", "bl_wheel_joint");
  declare_parameter<std::string>("br_wheel_joint", "br_wheel_joint");
  declare_parameter<std::string>("l_steer_joint", "l_steer_joint");
  declare_parameter<std::string>("r_steer_joint", "r_steer_joint");

  std::string robot_model;
  get_parameter("robot_model", robot_model);
  if (robot_model == "hunter_se") {
    wheelbase_ = HunterSEParams::wheelbase;
    track_ = HunterSEParams::track;
    wheel_radius_ = HunterSEParams::wheel_radius;
    reduction_ = HunterSEParams::transmission_reduction_rate;
  } else if (robot_model == "hunter1") {
    wheelbase_ = HunterV1Params::wheelbase;
    track_ = HunterV1Params::track;
    wheel_radius_ = HunterV1Params::wheel_radius;
    reduction_ = HunterV1Params::transmission_reduction_rate;
  } else {
    wheelbase_ = HunterV2Params::wheelbase;
    track_ = HunterV2Params::track;
    wheel_radius_ = HunterV2Params::wheel_radius;
    reduction_ = HunterV2Params::transmission_reduction_rate;
  }

  get_parameter("fl_wheel_joint", fl_wheel_joint_);
  get_parameter("fr_wheel_joint", fr_wheel_joint_);
  get_parameter("bl_wheel_joint", bl_wheel_joint_);
  get_parameter("br_wheel_joint", br_wheel_joint_);
  get_parameter("l_steer_joint", l_steer_joint_);
  get_parameter("r_steer_joint", r_steer_joint_);

  std::string status_topic;
  std::string joint_states_topic;
  get_parameter("status_topic", status_topic);
  get_parameter("joint_states_topic", joint_states_topic);

  pub_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);
  sub_ = create_subscription<hunter_msgs::msg::HunterStatus>(
      status_topic, 10,
      std::bind(&JointStatePublisher::OnStatus, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "joint_state_publisher started: model=%s wheelbase=%.3f "
              "track=%.3f wheel_radius=%.4f reduction=%.1f",
              robot_model.c_str(), wheelbase_, track_, wheel_radius_,
              reduction_);
}

void JointStatePublisher::SplitSteerAngle(double phi_c, double &phi_l,
                                          double &phi_r) const {
  phi_l = 0.0;
  phi_r = 0.0;
  if (phi_c > kSteerTol) {
    // left turn: left wheel is inner (larger angle)
    const double s = std::sin(phi_c);
    const double c = std::cos(phi_c);
    phi_l = std::atan2(2.0 * wheelbase_ * s,
                       2.0 * wheelbase_ * c - track_ * s);
    phi_r = std::atan2(2.0 * wheelbase_ * s,
                       2.0 * wheelbase_ * c + track_ * s);
  } else if (phi_c < -kSteerTol) {
    // right turn: right wheel is inner (larger magnitude, negative)
    const double p = -phi_c;
    const double s = std::sin(p);
    const double c = std::cos(p);
    phi_r = -std::atan2(2.0 * wheelbase_ * s,
                        2.0 * wheelbase_ * c - track_ * s);
    phi_l = -std::atan2(2.0 * wheelbase_ * s,
                        2.0 * wheelbase_ * c + track_ * s);
  }
}

double JointStatePublisher::MotorRpmToWheelOmega(double rpm) const {
  return rpm * 2.0 * M_PI / 60.0 / reduction_;
}

void JointStatePublisher::OnStatus(
    const hunter_msgs::msg::HunterStatus::SharedPtr msg) {
  double omega_br = 0.0;
  double omega_bl = 0.0;
  bool has_br = false;
  bool has_bl = false;
  for (const auto &act : msg->actuator_states) {
    const double omega = MotorRpmToWheelOmega(static_cast<double>(act.rpm));
    switch (act.motor_id) {
      case 2:  // rear right wheel
        omega_br = omega;
        has_br = true;
        break;
      case 3:  // rear left wheel
        omega_bl = omega;
        has_bl = true;
        break;
      default:
        break;
    }
  }

  // Fallback to chassis linear velocity if rear-wheel telemetry missing.
  const double omega_chassis = msg->linear_velocity / wheel_radius_;
  if (!has_br) omega_br = omega_chassis;
  if (!has_bl) omega_bl = omega_chassis;

  // Front wheels are not driven; estimate from Ackermann kinematics.
  // Each front wheel rolls at |omega_z| * d_i, where d_i is the distance
  // from the wheel center to the Instantaneous Center of Rotation (ICR).
  // ICR sits on the rear axle line at y = R = wheelbase / tan(phi_c).
  double omega_fl = omega_chassis;
  double omega_fr = omega_chassis;
  const double v = msg->linear_velocity;
  const double phi_c = msg->steering_angle;
  if (std::fabs(phi_c) >= kSteerTol) {
    const double R = wheelbase_ / std::tan(phi_c);   // signed turn radius
    const double d_fl = std::hypot(wheelbase_, R - track_ / 2.0);
    const double d_fr = std::hypot(wheelbase_, R + track_ / 2.0);
    const double sign_v = (v >= 0.0) ? 1.0 : -1.0;
    const double omega_z_abs = std::fabs(v / R);
    omega_fl = sign_v * omega_z_abs * d_fl / wheel_radius_;
    omega_fr = sign_v * omega_z_abs * d_fr / wheel_radius_;
  }

  // status.steering_angle is already the central (bicycle) angle.
  double phi_l = 0.0;
  double phi_r = 0.0;
  SplitSteerAngle(msg->steering_angle, phi_l, phi_r);

  // Integrate wheel angular velocities to get wheel angles for TF.
  // continuous joints in URDF use `position` (not velocity) for transform.
  const rclcpp::Time stamp(msg->header.stamp);
  double dt = 0.0;
  if (last_stamp_.nanoseconds() != 0) {
    dt = (stamp - last_stamp_).seconds();
    if (dt < 0.0 || dt > 1.0) dt = 0.0;  // skip on clock jump
  }
  last_stamp_ = stamp;

  pos_fl_ = std::remainder(pos_fl_ + omega_fl * dt, 2.0 * M_PI);
  pos_fr_ = std::remainder(pos_fr_ + omega_fr * dt, 2.0 * M_PI);
  pos_bl_ = std::remainder(pos_bl_ + omega_bl * dt, 2.0 * M_PI);
  pos_br_ = std::remainder(pos_br_ + omega_br * dt, 2.0 * M_PI);

  sensor_msgs::msg::JointState js;
  js.header.stamp = stamp;
  js.name = {fl_wheel_joint_, fr_wheel_joint_, bl_wheel_joint_,
             br_wheel_joint_, l_steer_joint_,  r_steer_joint_};
  js.position = {pos_fl_, pos_fr_, pos_bl_, pos_br_, phi_l, phi_r};
  js.velocity = {omega_fl, omega_fr, omega_bl, omega_br, 0.0, 0.0};
  pub_->publish(js);
}

}  // namespace westonrobot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
