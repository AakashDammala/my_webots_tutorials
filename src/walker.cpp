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
 * @file walker.cpp
 * @brief Implementation of the Walker robot node with state machine for obstacle avoidance.
 */

#include "my_webots_tutorials/walker.hpp"

#include <algorithm>
#include <cmath>

/// @brief ForwardState execute implementation - produces forward velocity command
geometry_msgs::msg::Twist ForwardState::execute()
{
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = speed_;
  twist.angular.z = 0.0;
  return twist;
}

/// @brief RotateState execute implementation - produces rotational velocity command
geometry_msgs::msg::Twist RotateState::execute()
{
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.0;
  twist.angular.z = direction_ * speed_;
  return twist;
}

/// @brief Walker constructor - initializes node, parameters, subscriptions, and state machine
Walker::Walker() : Node("walker"), is_rotating_(false)
{
  // Declare parameters
  this->declare_parameter<double>("forward_speed", 0.2);
  this->declare_parameter<double>("rotate_speed", 0.2);
  this->declare_parameter<double>("min_distance_threshold", 0.2);

  // Get parameter values
  forward_speed_ = this->get_parameter("forward_speed").as_double();
  rotate_speed_ = this->get_parameter("rotate_speed").as_double();
  min_distance_threshold_ = this->get_parameter("min_distance_threshold").as_double();

  // Initialize states
  rotate_state_ = std::make_unique<RotateState>(rotate_speed_);
  current_state_ = std::make_unique<ForwardState>(forward_speed_);

  // Create subscription to laser scan
  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Walker::scan_callback, this, std::placeholders::_1));

  // Create publisher for cmd_vel
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "Walker node initialized");
  RCLCPP_INFO(this->get_logger(), "Forward speed: %.2f, Rotate speed: %.2f, Min distance: %.2f",
              forward_speed_, rotate_speed_, min_distance_threshold_);
}

void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  size_t size = msg->ranges.size();

  if (size == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Received empty laser scan");
    return;
  }

  double dist = msg->ranges[10];

  if (std::isnan(dist) || std::isinf(dist)) dist = msg->range_max;

  RCLCPP_DEBUG(this->get_logger(), "Center scan dist: %.2f", dist);

  // Obstacle avoidance logic
  if (!is_rotating_ && dist <= min_distance_threshold_)
  {
    // Start rotating
    is_rotating_ = true;
    current_state_ = std::make_unique<RotateState>(*rotate_state_);
    RCLCPP_INFO(this->get_logger(), "Obstacle detected, starting rotation");
  }

  // Check if obstacle is cleared and start delay timer if not already started
  if (is_rotating_)
  {
    if (dist > min_distance_threshold_ * 2)
    {
      // Rotation complete, toggle direction and switch to forward
      rotate_state_->toggle_direction();
      current_state_ = std::make_unique<ForwardState>(forward_speed_);
      is_rotating_ = false;
      RCLCPP_INFO(this->get_logger(), "Rotation delay complete, switching direction");
    }
  }

  // Execute current state and publish command
  auto twist = current_state_->execute();
  cmd_vel_publisher_->publish(twist);
}

/// @brief Main entry point - initializes ROS2 and runs the Walker node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}
