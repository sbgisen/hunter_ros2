#include "hunter_base/hunter_joint_state_publisher.hpp"
#include "hunter_base/hunter_params.hpp"

#include <cmath>
#include <memory>

HunterJointStatePublisher::HunterJointStatePublisher()
: Node("hunter_joint_state_publisher"),
  first_message_(true)
{
  // Initialize wheel positions to zero
  wheel_positions_.resize(4, 0.0);

  // Declare joint name parameters
  this->declare_parameter("front_right_wheel_joint", "front_right_wheel");
  this->declare_parameter("front_left_wheel_joint", "front_left_wheel");
  this->declare_parameter("rear_right_wheel_joint", "rear_right_wheel");
  this->declare_parameter("rear_left_wheel_joint", "rear_left_wheel");
  this->declare_parameter("front_right_steering_joint", "front_right_steering");
  this->declare_parameter("front_left_steering_joint", "front_left_steering");

  // Get joint names
  front_right_wheel_joint_ = this->get_parameter("front_right_wheel_joint").as_string();
  front_left_wheel_joint_ = this->get_parameter("front_left_wheel_joint").as_string();
  rear_right_wheel_joint_ = this->get_parameter("rear_right_wheel_joint").as_string();
  rear_left_wheel_joint_ = this->get_parameter("rear_left_wheel_joint").as_string();
  front_right_steering_joint_ = this->get_parameter("front_right_steering_joint").as_string();
  front_left_steering_joint_ = this->get_parameter("front_left_steering_joint").as_string();

  // Create publisher for joint states
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

  // Create subscriber for hunter status
  hunter_status_sub_ = this->create_subscription<hunter_msgs::msg::HunterStatus>(
    "/hunter_status", 10,
    std::bind(&HunterJointStatePublisher::hunterStatusCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Hunter Joint State Publisher started");
  RCLCPP_INFO(this->get_logger(), "Wheel base: %.3f m, Track width: %.3f m",
              westonrobot::HunterV2Params::wheelbase,
              westonrobot::HunterV2Params::track);
  RCLCPP_INFO(this->get_logger(), "Joint names: [%s, %s, %s, %s, %s, %s]",
              front_right_wheel_joint_.c_str(), front_left_wheel_joint_.c_str(),
              rear_right_wheel_joint_.c_str(), rear_left_wheel_joint_.c_str(),
              front_right_steering_joint_.c_str(), front_left_steering_joint_.c_str());
}

void HunterJointStatePublisher::hunterStatusCallback(const hunter_msgs::msg::HunterStatus::SharedPtr msg)
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
    front_right_wheel_joint_,
    front_left_wheel_joint_,
    rear_right_wheel_joint_,
    rear_left_wheel_joint_,
    front_right_steering_joint_,
    front_left_steering_joint_,
  };

  // Initialize arrays with zeros
  joint_state_msg.position.resize(6, 0.0);
  joint_state_msg.velocity.resize(6, 0.0);
  joint_state_msg.effort.resize(6, 0.0);

  // Process actuator states to extract wheel velocities
  // The actuator_states array contains up to 3 motors
  for (size_t i = 0; i < std::min(msg->actuator_states.size(), size_t(3)); ++i) {
    const auto& actuator = msg->actuator_states[i];
    uint8_t motor_id = actuator.motor_id;  // Should match the index in joint_state_msg arrays

    // Convert RPM to rad/s: velocity = rpm * 2π / 60
    double velocity = actuator.rpm * 2.0 * M_PI / 60.0;

    // Map motor_id to joint index
    // MOTOR_ID_FRONT_RIGHT = 0, MOTOR_ID_FRONT_LEFT = 1
    // MOTOR_ID_REAR_RIGHT = 2, MOTOR_ID_REAR_LEFT = 3
    wheel_positions_[motor_id] += velocity * dt;
    joint_state_msg.position[motor_id] = wheel_positions_[motor_id];
    joint_state_msg.velocity[motor_id] = velocity;
    joint_state_msg.effort[motor_id] = actuator.current;
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

    constexpr double wheel_base = westonrobot::HunterV2Params::wheelbase;
    constexpr double track_width = westonrobot::HunterV2Params::track;
    double R = wheel_base / std::tan(central_angle);

    joint_state_msg.position[4] = std::atan(wheel_base / (R + track_width / 2.0));
    joint_state_msg.position[5] = std::atan(wheel_base / (R - track_width / 2.0));
  } else {
    // Going straight
    joint_state_msg.position[4] = 0.0;
    joint_state_msg.position[5] = 0.0;
  }

  // Publish joint states
  joint_state_pub_->publish(joint_state_msg);
}

auto main(int argc, char** argv) -> int
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HunterJointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
