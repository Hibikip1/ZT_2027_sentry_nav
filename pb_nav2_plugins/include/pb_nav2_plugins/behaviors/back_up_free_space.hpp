// Copyright 2024 Polaris Xia
// Copyright 2026 Jinbo Liu
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

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "pb_nav2_plugins/utils/local_esdf.hpp"
#include "rclcpp/rclcpp.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief ESDF-gradient-based escape behavior.
 *
 * When the robot is stuck inside an obstacle (cost >= 253), this behavior
 * computes the local ESDF, follows the distance-field gradient to escape,
 * and smoothly accelerates from a low speed to max_speed over ramp_duration.
 * It stops and returns SUCCEEDED once the ESDF distance exceeds a safety
 * threshold, indicating the robot has reached free space.
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  void onConfigure() override;
  void onCleanup() override;

  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  // Service client for obtaining the local costmap
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;

  // Lightweight local ESDF computed from the costmap message
  pb_nav2_plugins::LocalESDF esdf_;

  // Normalised escape direction (ESDF gradient direction)
  double escape_dir_x_{0.0};
  double escape_dir_y_{0.0};

  // Timestamp when the escape motion started (for smooth ramp-up)
  rclcpp::Time start_time_;

  // ---------- Parameters ----------
  std::string service_name_;
  double max_speed_{1.0};                // m/s – target escape speed
  double safe_distance_threshold_{0.2};  // m – stop when ESDF dist >= this
  double ramp_duration_{1.0};            // s – time to ramp from min to max speed

  static constexpr double MIN_SPEED = 0.1;  // m/s – initial escape speed
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
