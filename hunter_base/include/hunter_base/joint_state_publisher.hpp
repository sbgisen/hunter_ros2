/*
 * joint_state_publisher.hpp
 *
 * Description:
 *   Publishes sensor_msgs/JointState for the Hunter chassis by subscribing
 *   to /hunter_status. Computes all four wheel angular velocities (rad/s)
 *   and the front-left / front-right steering angles (rad) via Ackermann
 *   split.
 *
 *   Hunter SE actuator id mapping (from hardware):
 *     1: front steering motor (not a wheel velocity)
 *     2: rear right wheel motor
 *     3: rear left  wheel motor
 *   Front wheels are not driven, so their angular velocities are estimated
 *   from the chassis linear velocity.
 */

#ifndef HUNTER_BASE_JOINT_STATE_PUBLISHER_HPP
#define HUNTER_BASE_JOINT_STATE_PUBLISHER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hunter_msgs/msg/hunter_status.hpp"

namespace westonrobot {

class JointStatePublisher : public rclcpp::Node {
 public:
  JointStatePublisher();

 private:
  // ~0.29 deg dead-band for steering split
  static constexpr double kSteerTol = 0.005;

  void OnStatus(const hunter_msgs::msg::HunterStatus::SharedPtr msg);

  // Split a central (bicycle-model) steering angle into per-wheel angles
  // using Ackermann geometry. Sign convention: positive = left turn.
  void SplitSteerAngle(double phi_c, double &phi_l, double &phi_r) const;

  double MotorRpmToWheelOmega(double rpm) const;

  double wheelbase_ = 0.0;
  double track_ = 0.0;
  double wheel_radius_ = 0.0;
  double reduction_ = 1.0;

  // accumulated wheel rotations (rad), integrated from omega over time
  double pos_fl_ = 0.0;
  double pos_fr_ = 0.0;
  double pos_bl_ = 0.0;
  double pos_br_ = 0.0;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

  std::string fl_wheel_joint_;
  std::string fr_wheel_joint_;
  std::string bl_wheel_joint_;
  std::string br_wheel_joint_;
  std::string l_steer_joint_;
  std::string r_steer_joint_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<hunter_msgs::msg::HunterStatus>::SharedPtr sub_;
};

}  // namespace westonrobot

#endif  // HUNTER_BASE_JOINT_STATE_PUBLISHER_HPP
