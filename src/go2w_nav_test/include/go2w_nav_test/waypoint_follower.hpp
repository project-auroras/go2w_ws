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

#ifndef GO2W_NAV_TEST__WAYPOINT_FOLLOWER_HPP_
#define GO2W_NAV_TEST__WAYPOINT_FOLLOWER_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace go2w_nav_test
{

// Drives through a list of waypoints using only /odom, publishing /cmd_vel.
// Waypoints are relative to the pose where the robot is when the first /odom arrives.
// Stops while /scan sees an obstacle in front, or if /odom or /scan stop arriving.
class WaypointFollower : public rclcpp::Node
{
public:
  explicit WaypointFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_callback();
  void stop();

  // Parameters
  double max_linear_, max_angular_;
  double k_linear_, k_angular_;
  double goal_tolerance_;
  double heading_tolerance_;
  bool use_scan_;
  double stop_distance_;
  double obstacle_half_angle_;
  double data_timeout_;
  bool loop_;

  // Waypoints in the odom frame, computed from the start pose
  std::vector<std::pair<double, double>> relative_waypoints_;
  std::vector<std::pair<double, double>> waypoints_;
  size_t current_{0};

  bool have_odom_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double obstacle_distance_;
  bool obstacle_logged_{false};
  bool finished_{false};
  rclcpp::Time last_odom_;
  rclcpp::Time last_scan_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace go2w_nav_test

#endif  // GO2W_NAV_TEST__WAYPOINT_FOLLOWER_HPP_
