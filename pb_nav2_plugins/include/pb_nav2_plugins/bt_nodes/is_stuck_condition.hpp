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

#ifndef PB_NAV2_PLUGINS__BT_NODES__IS_STUCK_CONDITION_HPP_
#define PB_NAV2_PLUGINS__BT_NODES__IS_STUCK_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb_nav2_plugins
{

/**
 * @brief BT condition node that checks whether the robot is stuck inside an
 *        obstacle region on the local costmap.
 *
 * "Stuck" is defined as the robot's footprint centre having a costmap cost
 * >= 253 (INSCRIBED_INFLATED_OBSTACLE) for `consecutive_hits` consecutive
 * checks spaced at least `check_interval` seconds apart.
 *
 * This filtering prevents momentary lidar noise or brief contact from
 * triggering a full escape recovery.
 *
 * Returns SUCCESS when the robot is confirmed stuck, FAILURE otherwise.
 */
class IsStuckCondition : public BT::ConditionNode
{
public:
  IsStuckCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsStuckCondition() = delete;

  ~IsStuckCondition() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "costmap_topic", "local_costmap/costmap_raw", "Raw costmap topic to subscribe to"),
      BT::InputPort<std::string>(
        "robot_base_frame", "base_link", "Robot base frame used for TF lookup"),
      BT::InputPort<double>(
        "check_interval", 0.2, "Minimum seconds between consecutive stuck checks"),
      BT::InputPort<int>(
        "consecutive_hits", 3, "Number of consecutive high-cost checks required to confirm stuck"),
    };
  }

private:
  void costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;

  // Latest costmap (guarded by mutex_)
  std::mutex mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Parameters (read once from BT ports)
  std::string robot_base_frame_{"base_link"};
  double check_interval_{0.2};
  int consecutive_hits_{3};

  // Filtering state
  int hit_count_{0};
  rclcpp::Time last_check_time_;
  bool first_tick_{true};
};

}  // namespace pb_nav2_plugins

#endif  // PB_NAV2_PLUGINS__BT_NODES__IS_STUCK_CONDITION_HPP_
