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

#ifndef GO2W_DRIVER__SIM_GO2W_HPP_
#define GO2W_DRIVER__SIM_GO2W_HPP_

#include <array>

#include "rclcpp/rclcpp.hpp"

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace go2w_driver
{

// Kinematic stand-in for the Go2-W, to work without the robot.
// Publishes lowstate and sportmodestate like the robot does and executes
// /api/sport/request (Move, StopMove, StandUp, StandDown...), so go2w_driver
// runs unchanged on top of it. No physics, no LiDAR.
class SimGo2W : public rclcpp::Node
{
public:
  explicit SimGo2W(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void request_callback(const unitree_api::msg::Request::SharedPtr msg);
  void step();

  void set_pose_target(double thigh, double calf);
  // Height of base_link over the ground for a symmetric leg pose (hip = 0)
  double base_height(double thigh, double calf) const;

  // Parameters
  double period_;
  double thigh_length_, calf_length_, wheel_radius_, half_track_;
  double stand_thigh_, stand_calf_, down_thigh_, down_calf_;
  double transition_time_;
  double move_timeout_;

  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr request_sub_;
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Leg pose: interpolates from start to target in transition_time_
  double thigh_{0.0}, calf_{0.0};
  double thigh_start_{0.0}, calf_start_{0.0};
  double thigh_target_{0.0}, calf_target_{0.0};
  double transition_elapsed_{0.0};
  bool standing_{false};

  // Planar pose in odom and commanded body velocity
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double vx_{0.0}, vy_{0.0}, wz_{0.0};
  rclcpp::Time last_move_;

  std::array<double, 4> wheel_angles_{};
};

}  // namespace go2w_driver

#endif  // GO2W_DRIVER__SIM_GO2W_HPP_
