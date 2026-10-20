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
//
// Based on go2_driver by the Intelligent Robotics Lab (URJC),
// https://github.com/Unitree-Go2-Robot/go2_driver

#ifndef GO2W_DRIVER__GO2W_DRIVER_HPP_
#define GO2W_DRIVER__GO2W_DRIVER_HPP_

#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "go2_interfaces/srv/mode.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace go2w_driver
{

// Unitree sport API ids (same as Go2, see go2_driver/go2_api_id.hpp)
enum class SportApi : int64_t
{
  Damp = 1001,
  BalanceStand = 1002,
  StopMove = 1003,
  StandUp = 1004,
  StandDown = 1005,
  RecoveryStand = 1006,
  Move = 1008,
  Sit = 1009,
  RiseSit = 1010,
  Hello = 1016,
  Stretch = 1017,
};

// Bridges Unitree topics (unitree_go / unitree_api) to standard ROS 2 topics.
class Go2WDriver : public rclcpp::Node
{
public:
  explicit Go2WDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void low_state_callback(const unitree_go::msg::LowState::SharedPtr msg);
  void sport_state_callback(const unitree_go::msg::SportModeState::SharedPtr msg);
  void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void watchdog_callback();

  void handle_mode(
    const std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
    std::shared_ptr<go2_interfaces::srv::Mode::Response> response);

  // Publishes /odom and TF odom -> base_footprint -> base_link.
  // q is (w, x, y, z), v_body is the linear velocity in the base frame.
  void publish_odometry(
    const rclcpp::Time & stamp, const std::array<double, 3> & p,
    const std::array<double, 4> & q, const std::array<double, 3> & v_body, double wz);
  void send_request(SportApi api, const std::string & parameter = "");
  bool throttle(rclcpp::Time & last, const rclcpp::Time & now) const;

  // Parameters
  std::vector<std::string> joint_names_;
  std::vector<int64_t> motor_indices_;
  std::string odom_source_;
  std::string sport_state_topic_;
  std::string odom_frame_;
  std::string footprint_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_tf_;
  double base_height_offset_;
  double publish_period_;
  double max_vx_, max_vy_, max_wz_;
  double cmd_vel_timeout_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;

  rclcpp::Service<go2_interfaces::srv::Mode>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Time last_low_state_pub_;
  rclcpp::Time last_odom_pub_;
  rclcpp::Time last_battery_pub_;
  rclcpp::Time last_cmd_vel_;
  bool moving_{false};
};

}  // namespace go2w_driver

#endif  // GO2W_DRIVER__GO2W_DRIVER_HPP_
