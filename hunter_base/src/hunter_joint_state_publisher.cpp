#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hunter_msgs/msg/hunter_status.hpp"

class HunterJointStatePublisher : public rclcpp::Node
{
public:
  HunterJointStatePublisher()
  : Node("hunter_joint_state_publisher"),
    first_message_(true)
  {
    // Initialize wheel positions to zero
    wheel_positions_.resize(4, 0.0);

    // Declare parameters
    this->declare_parameter("wheel_base", 0.650);  // meters
    this->declare_parameter("track_width", 0.605);  // meters

    // Declare joint name parameters
    this->declare_parameter("front_left_wheel_joint", "front_left_wheel");
    this->declare_parameter("front_right_wheel_joint", "front_right_wheel");
    this->declare_parameter("rear_left_wheel_joint", "rear_left_wheel");
    this->declare_parameter("rear_right_wheel_joint", "rear_right_wheel");
    this->declare_parameter("front_left_steering_joint", "front_left_steering");
    this->declare_parameter("front_right_steering_joint", "front_right_steering");

    // Get parameters
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    track_width_ = this->get_parameter("track_width").as_double();

    // Get joint names
    front_left_wheel_joint_ = this->get_parameter("front_left_wheel_joint").as_string();
    front_right_wheel_joint_ = this->get_parameter("front_right_wheel_joint").as_string();
    rear_left_wheel_joint_ = this->get_parameter("rear_left_wheel_joint").as_string();
    rear_right_wheel_joint_ = this->get_parameter("rear_right_wheel_joint").as_string();
    front_left_steering_joint_ = this->get_parameter("front_left_steering_joint").as_string();
    front_right_steering_joint_ = this->get_parameter("front_right_steering_joint").as_string();

    // Create publisher for joint states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Create subscriber for hunter status
    hunter_status_sub_ = this->create_subscription<hunter_msgs::msg::HunterStatus>(
      "/hunter_status", 10,
      std::bind(&HunterJointStatePublisher::hunterStatusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hunter Joint State Publisher started");
    RCLCPP_INFO(this->get_logger(), "Wheel base: %.3f m, Track width: %.3f m",
                wheel_base_, track_width_);
    RCLCPP_INFO(this->get_logger(), "Joint names: [%s, %s, %s, %s, %s, %s]",
                front_left_wheel_joint_.c_str(), front_right_wheel_joint_.c_str(),
                rear_left_wheel_joint_.c_str(), rear_right_wheel_joint_.c_str(),
                front_left_steering_joint_.c_str(), front_right_steering_joint_.c_str());
  }

private:
  void hunterStatusCallback(const hunter_msgs::msg::HunterStatus::SharedPtr msg)
  {
    // Calculate dt for wheel position integration
    double dt = 0.0;
    if (!first_message_) {
      rclcpp::Time current_time(msg->header.stamp);
      rclcpp::Time last_time(last_stamp_);
      dt = (current_time - last_time).seconds();
    } else {
      first_message_ = false;
    }
    last_stamp_ = msg->header.stamp;

    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = msg->header.stamp;

    // Initialize joint names using parameters
    joint_state_msg.name = {
      front_left_wheel_joint_,
      front_right_wheel_joint_,
      rear_left_wheel_joint_,
      rear_right_wheel_joint_,
      front_left_steering_joint_,
      front_right_steering_joint_
    };

    // Initialize arrays with zeros
    joint_state_msg.position.resize(6, 0.0);
    joint_state_msg.velocity.resize(6, 0.0);
    joint_state_msg.effort.resize(6, 0.0);

    // Process actuator states to extract wheel velocities
    // The actuator_states array contains up to 3 motors
    for (size_t i = 0; i < std::min(msg->actuator_states.size(), size_t(3)); ++i) {
      const auto& actuator = msg->actuator_states[i];
      uint8_t motor_id = actuator.motor_id;

      // Convert RPM to rad/s: velocity = rpm * 2π / 60
      double velocity = actuator.rpm * 2.0 * M_PI / 60.0;

      // Map motor_id to joint index
      // MOTOR_ID_FRONT_RIGHT = 0, MOTOR_ID_FRONT_LEFT = 1
      // MOTOR_ID_REAR_RIGHT = 2, MOTOR_ID_REAR_LEFT = 3
      if (motor_id == 0) {  // Front right wheel
        wheel_positions_[1] += velocity * dt;
        joint_state_msg.position[1] = wheel_positions_[1];
        joint_state_msg.velocity[1] = velocity;
        joint_state_msg.effort[1] = actuator.current;
      } else if (motor_id == 1) {  // Front left wheel
        wheel_positions_[0] += velocity * dt;
        joint_state_msg.position[0] = wheel_positions_[0];
        joint_state_msg.velocity[0] = velocity;
        joint_state_msg.effort[0] = actuator.current;
      } else if (motor_id == 2) {  // Rear right wheel
        wheel_positions_[3] += velocity * dt;
        joint_state_msg.position[3] = wheel_positions_[3];
        joint_state_msg.velocity[3] = velocity;
        joint_state_msg.effort[3] = actuator.current;
      } else if (motor_id == 3) {  // Rear left wheel
        wheel_positions_[2] += velocity * dt;
        joint_state_msg.position[2] = wheel_positions_[2];
        joint_state_msg.velocity[2] = velocity;
        joint_state_msg.effort[2] = actuator.current;
      }
    }

    // Calculate steering angles using Ackermann geometry
    // The steering_angle from hunter_status is the central/equivalent steering angle
    double central_angle = msg->steering_angle;

    if (std::abs(central_angle) > 1e-6) {
      // For Ackermann steering, calculate individual wheel angles
      // R = turning radius
      // tan(central_angle) ≈ wheelbase / R (for small angles)
      // For the inner wheel: tan(inner) = wheelbase / (R - track/2)
      // For the outer wheel: tan(outer) = wheelbase / (R + track/2)

      double R = wheel_base_ / std::tan(central_angle);

      if (central_angle > 0) {  // Turning left
        // Left wheel is inner wheel
        joint_state_msg.position[4] = std::atan(wheel_base_ / (R - track_width_ / 2.0));
        // Right wheel is outer wheel
        joint_state_msg.position[5] = std::atan(wheel_base_ / (R + track_width_ / 2.0));
      } else {  // Turning right
        // Right wheel is inner wheel
        joint_state_msg.position[5] = std::atan(wheel_base_ / (R + track_width_ / 2.0));
        // Left wheel is outer wheel
        joint_state_msg.position[4] = std::atan(wheel_base_ / (R - track_width_ / 2.0));
      }
    } else {
      // Going straight
      joint_state_msg.position[4] = 0.0;
      joint_state_msg.position[5] = 0.0;
    }

    // Publish joint states
    joint_state_pub_->publish(joint_state_msg);
  }

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

  // State tracking for wheel position integration
  std::vector<double> wheel_positions_;  // Accumulated wheel positions [fl, fr, rl, rr]
  builtin_interfaces::msg::Time last_stamp_;
  bool first_message_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HunterJointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
