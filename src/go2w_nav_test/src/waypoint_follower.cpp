// Copyright 2026 AURORAS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "go2w_nav_test/waypoint_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace go2w_nav_test
{

using std::placeholders::_1;

WaypointFollower::WaypointFollower(const rclcpp::NodeOptions & options)
: Node("waypoint_follower", options),
  obstacle_distance_(std::numeric_limits<double>::infinity())
{
  // Flat list x0, y0, x1, y1, ... in meters, relative to the start pose
  // (x forward, y left). Default: 1 m square to the left.
  const auto flat = declare_parameter<std::vector<double>>(
    "waypoints", {1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0});

  max_linear_ = declare_parameter<double>("max_linear", 0.3);
  max_angular_ = declare_parameter<double>("max_angular", 0.5);
  k_linear_ = declare_parameter<double>("k_linear", 0.5);
  k_angular_ = declare_parameter<double>("k_angular", 1.5);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.10);
  // Turn in place while the heading error is larger than this
  heading_tolerance_ = declare_parameter<double>("heading_tolerance", 0.3);
  use_scan_ = declare_parameter<bool>("use_scan", false);
  stop_distance_ = declare_parameter<double>("stop_distance", 0.5);
  obstacle_half_angle_ = declare_parameter<double>("obstacle_half_angle", 0.5);
  data_timeout_ = declare_parameter<double>("data_timeout", 0.5);
  loop_ = declare_parameter<bool>("loop", false);
  const double rate = declare_parameter<double>("control_rate", 20.0);

  if (flat.empty() || flat.size() % 2 != 0) {
    throw std::invalid_argument("waypoints must be a non-empty list of x, y pairs");
  }
  for (size_t i = 0; i < flat.size(); i += 2) {
    relative_waypoints_.emplace_back(flat[i], flat[i + 1]);
  }

  const auto zero = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_odom_ = last_scan_ = zero;

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&WaypointFollower::odom_callback, this, _1));
  if (use_scan_) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&WaypointFollower::scan_callback, this, _1));
  }
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&WaypointFollower::control_callback, this));

  RCLCPP_INFO(
    get_logger(), "%zu waypoints, max %.2f m/s %.2f rad/s, obstacle stop %s at %.2f m",
    relative_waypoints_.size(), max_linear_, max_angular_, use_scan_ ? "on" : "off",
    stop_distance_);
}

void WaypointFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  last_odom_ = now();

  if (!have_odom_) {
    // Waypoints are given in the start pose frame: move them to odom
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    for (const auto & [wx, wy] : relative_waypoints_) {
      waypoints_.emplace_back(x_ + c * wx - s * wy, y_ + s * wx + c * wy);
    }
    have_odom_ = true;
    RCLCPP_INFO(get_logger(), "Start pose (%.2f, %.2f, %.2f rad)", x_, y_, yaw_);
  }
}

void WaypointFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Closest valid range in the front sector (scan frame x forward)
  double closest = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    const double angle = std::remainder(msg->angle_min + i * msg->angle_increment, 2.0 * M_PI);
    const double range = msg->ranges[i];
    if (std::abs(angle) <= obstacle_half_angle_ && std::isfinite(range) &&
      range >= msg->range_min && range <= msg->range_max)
    {
      closest = std::min(closest, range);
    }
  }
  obstacle_distance_ = closest;
  last_scan_ = now();
}

void WaypointFollower::stop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void WaypointFollower::control_callback()
{
  if (!have_odom_ || finished_) {
    return;
  }

  const auto t = now();
  if ((t - last_odom_).seconds() > data_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No /odom, stopped");
    stop();
    return;
  }
  if (use_scan_ && (t - last_scan_).seconds() > data_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No /scan, stopped");
    stop();
    return;
  }

  const auto [gx, gy] = waypoints_[current_];
  const double dx = gx - x_, dy = gy - y_;
  const double distance = std::hypot(dx, dy);

  if (distance < goal_tolerance_) {
    RCLCPP_INFO(
      get_logger(), "Waypoint %zu/%zu reached (%.2f, %.2f)", current_ + 1, waypoints_.size(),
      gx, gy);
    current_++;
    if (current_ == waypoints_.size()) {
      if (!loop_) {
        RCLCPP_INFO(get_logger(), "All waypoints reached");
        finished_ = true;
        stop();
        return;
      }
      current_ = 0;
    }
    return;
  }

  const double heading_error = std::remainder(std::atan2(dy, dx) - yaw_, 2.0 * M_PI);

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = std::clamp(k_angular_ * heading_error, -max_angular_, max_angular_);
  if (std::abs(heading_error) < heading_tolerance_) {
    cmd.linear.x = std::min(k_linear_ * distance, max_linear_) * std::cos(heading_error);
  }

  if (use_scan_ && obstacle_distance_ < stop_distance_) {
    if (!obstacle_logged_) {
      RCLCPP_WARN(get_logger(), "Obstacle at %.2f m, waiting", obstacle_distance_);
      obstacle_logged_ = true;
    }
    stop();
    return;
  }
  if (obstacle_logged_) {
    RCLCPP_INFO(get_logger(), "Path clear, resuming");
    obstacle_logged_ = false;
  }

  cmd_vel_pub_->publish(cmd);
}

}  // namespace go2w_nav_test

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2w_nav_test::WaypointFollower)
