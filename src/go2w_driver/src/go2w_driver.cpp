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

#include "go2w_driver/go2w_driver.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace go2w_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

Go2WDriver::Go2WDriver(const rclcpp::NodeOptions & options)
: Node("go2w_driver", options),
  tf_broadcaster_(this)
{
  // Unitree motor order: FR, FL, RR, RL (hip, thigh, calf), then the four wheels.
  // Wheel indices 12-15 checked on the real robot.
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names", {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      "FR_foot_joint", "FL_foot_joint", "RR_foot_joint", "RL_foot_joint"});
  motor_indices_ = declare_parameter<std::vector<int64_t>>(
    "motor_indices", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});

  odom_source_ = declare_parameter<std::string>("odom_source", "sportmodestate");
  sport_state_topic_ = declare_parameter<std::string>("sport_state_topic", "sportmodestate");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  footprint_frame_ = declare_parameter<std::string>("footprint_frame", "base_footprint");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  base_height_offset_ = declare_parameter<double>("base_height_offset", 0.0);
  publish_period_ = 1.0 / declare_parameter<double>("publish_rate", 100.0);

  max_vx_ = declare_parameter<double>("max_vx", 1.0);
  max_vy_ = declare_parameter<double>("max_vy", 0.0);
  max_wz_ = declare_parameter<double>("max_wz", 1.5);
  cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);

  if (joint_names_.size() != motor_indices_.size()) {
    throw std::invalid_argument("joint_names and motor_indices must have the same size");
  }
  const int64_t num_motors = unitree_go::msg::LowState().motor_state.size();
  for (const auto index : motor_indices_) {
    if (index < 0 || index >= num_motors) {
      throw std::invalid_argument("motor index out of range: " + std::to_string(index));
    }
  }

  const auto zero = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_low_state_pub_ = last_odom_pub_ = last_battery_pub_ = last_cmd_vel_ = zero;

  // Unitree publishers may be best effort: a best-effort subscription matches both.
  const auto unitree_qos = rclcpp::SensorDataQoS();

  low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
    "lowstate", unitree_qos, std::bind(&Go2WDriver::low_state_callback, this, _1));

  if (odom_source_ == "sportmodestate") {
    sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_state_topic_, unitree_qos, std::bind(&Go2WDriver::sport_state_callback, this, _1));
  } else if (odom_source_ == "robot_pose") {
    robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/utlidar/robot_pose", unitree_qos,
      std::bind(&Go2WDriver::robot_pose_callback, this, _1));
  } else {
    throw std::invalid_argument("odom_source must be 'sportmodestate' or 'robot_pose'");
  }

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&Go2WDriver::cmd_vel_callback, this, _1));

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  request_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

  mode_service_ = create_service<go2_interfaces::srv::Mode>(
    "mode", std::bind(&Go2WDriver::handle_mode, this, _1, _2));

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&Go2WDriver::watchdog_callback, this));

  RCLCPP_INFO(
    get_logger(), "go2w_driver started (odom_source: %s%s)", odom_source_.c_str(),
    sport_state_sub_ ? (", " + sport_state_topic_).c_str() : "");
}

bool Go2WDriver::throttle(rclcpp::Time & last, const rclcpp::Time & now) const
{
  if ((now - last).seconds() < publish_period_) {
    return false;
  }
  last = now;
  return true;
}

void Go2WDriver::low_state_callback(const unitree_go::msg::LowState::SharedPtr msg)
{
  const auto stamp = now();
  if (!throttle(last_low_state_pub_, stamp)) {
    return;
  }

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name = joint_names_;
  for (const auto index : motor_indices_) {
    const auto & motor = msg->motor_state.at(index);
    joint_state.position.push_back(motor.q);
    joint_state.velocity.push_back(motor.dq);
    joint_state.effort.push_back(motor.tau_est);
  }
  joint_state_pub_->publish(joint_state);

  // imu_state.quaternion is (w, x, y, z)
  const auto & imu_state = msg->imu_state;
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = imu_frame_;
  imu.orientation.w = imu_state.quaternion[0];
  imu.orientation.x = imu_state.quaternion[1];
  imu.orientation.y = imu_state.quaternion[2];
  imu.orientation.z = imu_state.quaternion[3];
  imu.angular_velocity.x = imu_state.gyroscope[0];
  imu.angular_velocity.y = imu_state.gyroscope[1];
  imu.angular_velocity.z = imu_state.gyroscope[2];
  imu.linear_acceleration.x = imu_state.accelerometer[0];
  imu.linear_acceleration.y = imu_state.accelerometer[1];
  imu.linear_acceleration.z = imu_state.accelerometer[2];
  for (int i = 0; i < 3; i++) {
    imu.orientation_covariance[i * 4] = 1e-3;
    imu.angular_velocity_covariance[i * 4] = 1e-4;
    imu.linear_acceleration_covariance[i * 4] = 1e-2;
  }
  imu_pub_->publish(imu);

  if ((stamp - last_battery_pub_).seconds() >= 1.0) {
    last_battery_pub_ = stamp;
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = stamp;
    battery.voltage = msg->power_v;
    battery.current = msg->power_a;
    battery.percentage = msg->bms_state.soc / 100.0f;
    battery.present = true;
    battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_pub_->publish(battery);
  }
}

void Go2WDriver::sport_state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
  const auto stamp = now();
  if (!throttle(last_odom_pub_, stamp)) {
    return;
  }

  const auto & q = msg->imu_state.quaternion;
  const std::array<double, 4> orientation{q[0], q[1], q[2], q[3]};

  // SportModeState velocity is expressed in the odom (world) frame
  const tf2::Matrix3x3 rotation(tf2::Quaternion(q[1], q[2], q[3], q[0]));
  const tf2::Vector3 v_body =
    rotation.transpose() * tf2::Vector3(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

  publish_odometry(
    stamp, {msg->position[0], msg->position[1], msg->position[2]}, orientation,
    {v_body.x(), v_body.y(), v_body.z()}, msg->yaw_speed);
}

void Go2WDriver::robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto stamp = now();
  if (!throttle(last_odom_pub_, stamp)) {
    return;
  }

  // robot_pose carries no velocity
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  publish_odometry(stamp, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}, {0.0, 0.0, 0.0}, 0.0);
}

void Go2WDriver::publish_odometry(
  const rclcpp::Time & stamp, const std::array<double, 3> & p,
  const std::array<double, 4> & q, const std::array<double, 3> & v_body, double wz)
{
  const tf2::Quaternion orientation(q[1], q[2], q[3], q[0]);
  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0.0, 0.0, yaw);
  tf2::Quaternion tilt;
  tilt.setRPY(roll, pitch, 0.0);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = p[0];
  odom.pose.pose.position.y = p[1];
  odom.pose.pose.position.z = p[2] + base_height_offset_;
  odom.pose.pose.orientation.x = q[1];
  odom.pose.pose.orientation.y = q[2];
  odom.pose.pose.orientation.z = q[3];
  odom.pose.pose.orientation.w = q[0];
  odom.twist.twist.linear.x = v_body[0];
  odom.twist.twist.linear.y = v_body[1];
  odom.twist.twist.linear.z = v_body[2];
  odom.twist.twist.angular.z = wz;
  for (int i = 0; i < 6; i++) {
    odom.pose.covariance[i * 7] = 1e-2;
    odom.twist.covariance[i * 7] = robot_pose_sub_ ? 1e3 : 1e-2;
  }
  odom_pub_->publish(odom);

  if (!publish_tf_) {
    return;
  }

  geometry_msgs::msg::TransformStamped odom_to_footprint;
  odom_to_footprint.header.stamp = stamp;
  odom_to_footprint.header.frame_id = odom_frame_;
  odom_to_footprint.child_frame_id = footprint_frame_;
  odom_to_footprint.transform.translation.x = p[0];
  odom_to_footprint.transform.translation.y = p[1];
  odom_to_footprint.transform.rotation.x = yaw_only.x();
  odom_to_footprint.transform.rotation.y = yaw_only.y();
  odom_to_footprint.transform.rotation.z = yaw_only.z();
  odom_to_footprint.transform.rotation.w = yaw_only.w();

  geometry_msgs::msg::TransformStamped footprint_to_base;
  footprint_to_base.header.stamp = stamp;
  footprint_to_base.header.frame_id = footprint_frame_;
  footprint_to_base.child_frame_id = base_frame_;
  footprint_to_base.transform.translation.z = p[2] + base_height_offset_;
  footprint_to_base.transform.rotation.x = tilt.x();
  footprint_to_base.transform.rotation.y = tilt.y();
  footprint_to_base.transform.rotation.z = tilt.z();
  footprint_to_base.transform.rotation.w = tilt.w();

  tf_broadcaster_.sendTransform({odom_to_footprint, footprint_to_base});
}

void Go2WDriver::send_request(SportApi api, const std::string & parameter)
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = static_cast<int64_t>(api);
  req.parameter = parameter;
  request_pub_->publish(req);
}

void Go2WDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nlohmann::json js;
  js["x"] = std::clamp(msg->linear.x, -max_vx_, max_vx_);
  js["y"] = std::clamp(msg->linear.y, -max_vy_, max_vy_);
  js["z"] = std::clamp(msg->angular.z, -max_wz_, max_wz_);
  send_request(SportApi::Move, js.dump());

  last_cmd_vel_ = now();
  moving_ = true;
}

void Go2WDriver::watchdog_callback()
{
  if (!moving_ || (now() - last_cmd_vel_).seconds() < cmd_vel_timeout_) {
    return;
  }

  RCLCPP_WARN(get_logger(), "No cmd_vel for %.2f s, stopping", cmd_vel_timeout_);
  send_request(SportApi::Move, R"({"x": 0.0, "y": 0.0, "z": 0.0})");
  moving_ = false;
}

void Go2WDriver::handle_mode(
  const std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
  std::shared_ptr<go2_interfaces::srv::Mode::Response> response)
{
  static const std::map<std::string, SportApi> modes = {
    {"damp", SportApi::Damp},
    {"balance_stand", SportApi::BalanceStand},
    {"stop_move", SportApi::StopMove},
    {"stand_up", SportApi::StandUp},
    {"stand_down", SportApi::StandDown},
    {"recovery_stand", SportApi::RecoveryStand},
    {"sit", SportApi::Sit},
    {"rise_sit", SportApi::RiseSit},
    {"hello", SportApi::Hello},
    {"stretch", SportApi::Stretch},
  };

  const auto it = modes.find(request->mode);
  if (it == modes.end()) {
    response->success = false;
    response->message = "Unknown mode '" + request->mode + "'";
    return;
  }

  send_request(it->second);
  response->success = true;
  response->message = "Mode set to " + request->mode;
}

}  // namespace go2w_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2w_driver::Go2WDriver)
