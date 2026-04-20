#ifndef HUNTER_JOINT_STATE_PUBLISHER_HPP
#define HUNTER_JOINT_STATE_PUBLISHER_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hunter_msgs/msg/hunter_status.hpp"

class HunterJointStatePublisher : public rclcpp::Node
{
public:
  HunterJointStatePublisher();

private:
  void hunterStatusCallback(const hunter_msgs::msg::HunterStatus::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<hunter_msgs::msg::HunterStatus>::SharedPtr hunter_status_sub_;

  double wheel_base_;
  double track_width_;

  std::string front_left_wheel_joint_;
  std::string front_right_wheel_joint_;
  std::string rear_left_wheel_joint_;
  std::string rear_right_wheel_joint_;
  std::string front_left_steering_joint_;
  std::string front_right_steering_joint_;

  std::vector<double> wheel_positions_;
  builtin_interfaces::msg::Time last_stamp_;
  bool first_message_;
};

#endif  // HUNTER_JOINT_STATE_PUBLISHER_HPP
