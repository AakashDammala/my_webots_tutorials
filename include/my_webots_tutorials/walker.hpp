// MIT License
//
// Copyright (c) 2025 Aakash Dammala
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file walker.hpp
 * @brief Walker robot node implementation with state machine for obstacle avoidance.
 */

#ifndef WALKER_HPP
#define WALKER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @class State
 * @brief Abstract base class defining the interface for walker states.
 *
 * Uses the State design pattern to implement different behaviors (Forward, Rotate).
 * Each concrete state implements the execute() method to return appropriate velocity commands.
 */
class State
{
 public:
  /// Virtual destructor for proper cleanup of derived classes
  virtual ~State() = default;
  
  /**
   * @brief Execute the state and return the corresponding velocity command.
   * @return geometry_msgs::msg::Twist The velocity command to be published.
   */
  virtual geometry_msgs::msg::Twist execute() = 0;
};

/**
 * @class ForwardState
 * @brief Concrete state for forward movement.
 *
 * Inherits from State. When executed, produces a velocity command to move the robot forward
 * with zero angular velocity.
 */
class ForwardState : public State
{
 private:
  double speed_;  ///< Linear velocity in m/s

 public:
  /**
   * @brief Constructor for ForwardState.
   * @param speed Linear velocity in m/s for forward movement.
   */
  ForwardState(double speed) : speed_(speed) {}
  
  /**
   * @brief Execute forward movement.
   * @return geometry_msgs::msg::Twist Velocity command with linear motion, zero rotation.
   */
  geometry_msgs::msg::Twist execute() override;
};

/**
 * @class RotateState
 * @brief Concrete state for in-place rotation.
 *
 * Inherits from State. When executed, produces a velocity command to rotate the robot in place
 * with zero linear velocity. Supports alternating rotation directions (clockwise/counterclockwise).
 */
class RotateState : public State
{
 private:
  double speed_;      ///< Angular velocity in rad/s
  int direction_;     ///< Rotation direction: 1 for counterclockwise, -1 for clockwise
 
 public:
  /**
   * @brief Constructor for RotateState.
   * @param speed Angular velocity in rad/s for rotation.
   */
  RotateState(double speed) : speed_(speed), direction_(1) {}
  
  /**
   * @brief Execute rotation movement.
   * @return geometry_msgs::msg::Twist Velocity command with angular motion, zero linear.
   */
  geometry_msgs::msg::Twist execute() override;
  
  /**
   * @brief Toggle rotation direction between clockwise and counterclockwise.
   * Multiplies direction by -1 to alternate between left and right rotation.
   */
  void toggle_direction() { direction_ *= -1; }
};

/**
 * @class Walker
 * @brief ROS2 node implementing a simple robot walker with obstacle avoidance.
 *
 * The Walker node uses the State design pattern to manage two behaviors:
 * - Moving forward until an obstacle is detected via laser scan
 * - Rotating in place (alternating directions) when an obstacle is detected
 *
 * The robot subscribes to /scan (laser scan) and publishes to /cmd_vel (velocity commands).
 */
class Walker : public rclcpp::Node
{
 public:
  /**
   * @brief Constructor for the Walker node.
   * Initializes ROS2 node, sets up subscriptions, publishers, and state machine.
   */
  Walker();

 private:
  /**
   * @brief Callback function for laser scan messages.
   *
   * Implements the obstacle avoidance logic:
   * - Detects obstacles based on minimum distance threshold
   * - Transitions between Forward and Rotate states
   * - Publishes velocity commands based on current state
   *
   * @param msg Shared pointer to the received LaserScan message.
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;  ///< Subscription to /scan topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;       ///< Publisher to /cmd_vel topic

  // State management
  std::unique_ptr<State> current_state_;    ///< Current active state (Forward or Rotate)
  std::unique_ptr<RotateState> rotate_state_;  ///< Reference to rotate state for direction toggling
  bool is_rotating_;                        ///< Flag indicating if robot is currently rotating

  // Parameters for obstacle avoidance
  double forward_speed_;          ///< Linear velocity for forward movement (m/s)
  double rotate_speed_;           ///< Angular velocity for rotation (rad/s)
  double min_distance_threshold_; ///< Minimum safe distance before obstacle detected (m)
};

#endif  // WALKER_HPP
