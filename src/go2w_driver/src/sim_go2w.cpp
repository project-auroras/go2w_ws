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

#include "go2w_driver/sim_go2w.hpp"

#include <algorithm>
#include <cmath>

#include "nlohmann/json.hpp"

#include "go2w_driver/go2w_driver.hpp"

namespace go2w_driver
{

using std::placeholders::_1;

SimGo2W::SimGo2W(const rclcpp::NodeOptions & options)
: Node("sim_go2w", options)
{
  period_ = 1.0 / declare_parameter<double>("publish_rate", 100.0);

  // Leg and wheel geometry from go2w_description
  thigh_length_ = declare_parameter<double>("thigh_length", 0.213);
  calf_length_ = declare_parameter<double>("calf_length", 0.2264);
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.086);
  half_track_ = declare_parameter<double>("half_track", 0.15);

  stand_thigh_ = declare_parameter<double>("stand_thigh", 0.67);
  stand_calf_ = declare_parameter<double>("stand_calf", -1.3);
  down_thigh_ = declare_parameter<double>("down_thigh", 1.22);
  down_calf_ = declare_parameter<double>("down_calf", -2.72);
  transition_time_ = declare_parameter<double>("transition_time", 1.0);
  move_timeout_ = declare_parameter<double>("move_timeout", 1.0);

  standing_ = declare_parameter<bool>("start_standing", true);
  thigh_ = thigh_start_ = thigh_target_ = standing_ ? stand_thigh_ : down_thigh_;
  calf_ = calf_start_ = calf_target_ = standing_ ? stand_calf_ : down_calf_;
  transition_elapsed_ = transition_time_;

  last_move_ = now();

  request_sub_ = create_subscription<unitree_api::msg::Request>(
    "/api/sport/request", 10, std::bind(&SimGo2W::request_callback, this, _1));

  // Same QoS as the robot
  low_state_pub_ = create_publisher<unitree_go::msg::LowState>(
    "lowstate", rclcpp::SensorDataQoS());
  sport_state_pub_ = create_publisher<unitree_go::msg::SportModeState>(
    "sportmodestate", rclcpp::SensorDataQoS());

  timer_ = create_wall_timer(
    std::chrono::duration<double>(period_), std::bind(&SimGo2W::step, this));

  RCLCPP_WARN(get_logger(), "Simulated Go2-W running: do not use with the real robot connected");
}

double SimGo2W::base_height(double thigh, double calf) const
{
  return thigh_length_ * std::cos(thigh) + calf_length_ * std::cos(thigh + calf) +
         wheel_radius_;
}

void SimGo2W::set_pose_target(double thigh, double calf)
{
  thigh_start_ = thigh_;
  calf_start_ = calf_;
  thigh_target_ = thigh;
  calf_target_ = calf;
  transition_elapsed_ = 0.0;
}

void SimGo2W::request_callback(const unitree_api::msg::Request::SharedPtr msg)
{
  const auto api = static_cast<SportApi>(msg->header.identity.api_id);

  switch (api) {
    case SportApi::Move: {
      if (!standing_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Move ignored: not standing");
        return;
      }
      const auto js = nlohmann::json::parse(msg->parameter, nullptr, false);
      if (js.is_discarded()) {
        RCLCPP_ERROR(get_logger(), "Move with invalid parameter: %s", msg->parameter.c_str());
        return;
      }
      vx_ = js.value("x", 0.0);
      vy_ = js.value("y", 0.0);
      wz_ = js.value("z", 0.0);
      last_move_ = now();
      break;
    }
    case SportApi::StopMove:
      vx_ = vy_ = wz_ = 0.0;
      break;
    case SportApi::StandUp:
    case SportApi::RecoveryStand:
    case SportApi::BalanceStand:
    case SportApi::RiseSit:
      standing_ = true;
      set_pose_target(stand_thigh_, stand_calf_);
      break;
    case SportApi::StandDown:
    case SportApi::Damp:
      standing_ = false;
      vx_ = vy_ = wz_ = 0.0;
      set_pose_target(down_thigh_, down_calf_);
      break;
    default:
      RCLCPP_INFO(get_logger(), "api_id %ld not simulated", msg->header.identity.api_id);
      return;
  }
  RCLCPP_DEBUG(get_logger(), "api_id %ld", msg->header.identity.api_id);
}

void SimGo2W::step()
{
  // Like the robot, stop if Move is not refreshed
  if ((now() - last_move_).seconds() > move_timeout_) {
    vx_ = vy_ = wz_ = 0.0;
  }

  if (transition_elapsed_ < transition_time_) {
    transition_elapsed_ = std::min(transition_elapsed_ + period_, transition_time_);
    const double s = transition_time_ > 0.0 ? transition_elapsed_ / transition_time_ : 1.0;
    thigh_ = thigh_start_ + s * (thigh_target_ - thigh_start_);
    calf_ = calf_start_ + s * (calf_target_ - calf_start_);
  }

  // Integrate the body velocity in the odom frame
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const double vx_world = c * vx_ - s * vy_;
  const double vy_world = s * vx_ + c * vy_;
  x_ += vx_world * period_;
  y_ += vy_world * period_;
  yaw_ = std::remainder(yaw_ + wz_ * period_, 2.0 * M_PI);

  // Differential drive wheel speeds, FR, FL, RR, RL
  const double right = (vx_ + wz_ * half_track_) / wheel_radius_;
  const double left = (vx_ - wz_ * half_track_) / wheel_radius_;
  const std::array<double, 4> wheel_speeds{right, left, right, left};
  for (size_t i = 0; i < wheel_speeds.size(); i++) {
    wheel_angles_[i] = std::remainder(wheel_angles_[i] + wheel_speeds[i] * period_, 2.0 * M_PI);
  }

  // imu_state.quaternion is (w, x, y, z), yaw only
  unitree_go::msg::IMUState imu;
  imu.quaternion = {
    static_cast<float>(std::cos(yaw_ / 2.0)), 0.0f, 0.0f,
    static_cast<float>(std::sin(yaw_ / 2.0))};
  imu.gyroscope = {0.0f, 0.0f, static_cast<float>(wz_)};
  imu.accelerometer = {0.0f, 0.0f, 9.81f};
  imu.rpy = {0.0f, 0.0f, static_cast<float>(yaw_)};

  // Motor order: FR, FL, RR, RL (hip, thigh, calf), then the wheels
  unitree_go::msg::LowState low_state;
  low_state.imu_state = imu;
  for (int leg = 0; leg < 4; leg++) {
    low_state.motor_state[leg * 3 + 1].q = thigh_;
    low_state.motor_state[leg * 3 + 2].q = calf_;
    low_state.motor_state[12 + leg].q = wheel_angles_[leg];
    low_state.motor_state[12 + leg].dq = wheel_speeds[leg];
  }
  low_state.power_v = 25.0f;
  low_state.bms_state.soc = 100;
  low_state_pub_->publish(low_state);

  const double height = base_height(thigh_, calf_);
  unitree_go::msg::SportModeState sport_state;
  sport_state.imu_state = imu;
  sport_state.position = {
    static_cast<float>(x_), static_cast<float>(y_), static_cast<float>(height)};
  sport_state.body_height = static_cast<float>(height);
  sport_state.velocity = {static_cast<float>(vx_world), static_cast<float>(vy_world), 0.0f};
  sport_state.yaw_speed = static_cast<float>(wz_);
  sport_state_pub_->publish(sport_state);
}

}  // namespace go2w_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2w_driver::SimGo2W)
