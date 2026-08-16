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

#include "pb_nav2_plugins/bt_nodes/is_stuck_condition.hpp"

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

namespace pb_nav2_plugins
{

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

IsStuckCondition::IsStuckCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), first_tick_(true)
{
  // Obtain the ROS node handle from the BT blackboard (set by bt_navigator)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Read BT port values
  std::string costmap_topic;
  getInput("costmap_topic", costmap_topic);
  getInput("check_interval", check_interval_);
  getInput("consecutive_hits", consecutive_hits_);

  // Set up a dedicated callback group so the subscription callback runs on
  // its own executor thread – identical pattern to Nav2's IsBatteryLow and
  // IsStuckCondition nodes.
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, rclcpp::SensorDataQoS(),
    std::bind(&IsStuckCondition::costmapCallback, this, std::placeholders::_1), sub_options);

  // Spin the callback group executor in a background thread
  callback_group_executor_thread_ = std::thread([this]() { callback_group_executor_.spin(); });

  RCLCPP_INFO(
    node_->get_logger(), "IsStuckCondition: listening on '%s', interval=%.2fs, hits=%d",
    costmap_topic.c_str(), check_interval_, consecutive_hits_);
}

IsStuckCondition::~IsStuckCondition()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Costmap callback (runs on the executor thread)
// ---------------------------------------------------------------------------

void IsStuckCondition::costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_costmap_ = msg;
}

// ---------------------------------------------------------------------------
// tick()
// ---------------------------------------------------------------------------

BT::NodeStatus IsStuckCondition::tick()
{
  // ---- 1. Grab the latest costmap snapshot ----
  nav2_msgs::msg::Costmap::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    costmap = latest_costmap_;
  }

  if (!costmap) {
    // No costmap received yet – cannot determine stuck status
    return BT::NodeStatus::FAILURE;
  }

  // ---- 2. Get the robot pose in the costmap frame ----
  // The costmap metadata provides the frame and grid parameters.
  // We use TF to obtain the robot pose in the costmap's frame_id.
  geometry_msgs::msg::TransformStamped transform;
  try {
    auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    transform =
      tf_buffer->lookupTransform(costmap->header.frame_id, "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "IsStuckCondition: TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;

  // ---- 3. Convert world coords to grid indices ----
  const double origin_x = costmap->metadata.origin.position.x;
  const double origin_y = costmap->metadata.origin.position.y;
  const double resolution = costmap->metadata.resolution;
  const unsigned int size_x = costmap->metadata.size_x;
  const unsigned int size_y = costmap->metadata.size_y;

  const int mx = static_cast<int>((robot_x - origin_x) / resolution);
  const int my = static_cast<int>((robot_y - origin_y) / resolution);

  if (
    mx < 0 || my < 0 || static_cast<unsigned int>(mx) >= size_x ||
    static_cast<unsigned int>(my) >= size_y) {
    // Robot is outside the costmap – reset and return FAILURE
    hit_count_ = 0;
    first_tick_ = true;
    return BT::NodeStatus::FAILURE;
  }

  // ---- 4. Read cost at the robot's cell ----
  const unsigned int idx = static_cast<unsigned int>(my) * size_x + static_cast<unsigned int>(mx);
  const uint8_t cost = costmap->data[idx];

  // 253 = INSCRIBED_INFLATED_OBSTACLE
  const bool is_in_obstacle = (cost >= 253);

  // ---- 5. Temporal filtering ----
  const rclcpp::Time now = node_->get_clock()->now();

  if (!is_in_obstacle) {
    // Robot is NOT in an obstacle region – reset the counter
    hit_count_ = 0;
    first_tick_ = true;
    return BT::NodeStatus::FAILURE;
  }

  // Robot IS in an obstacle region (cost >= 253)
  if (first_tick_) {
    // First detection – start the timer
    hit_count_ = 1;
    last_check_time_ = now;
    first_tick_ = false;
    return BT::NodeStatus::FAILURE;
  }

  // Check if enough time has passed since the last counted hit
  const double elapsed = (now - last_check_time_).seconds();
  if (elapsed >= check_interval_) {
    hit_count_++;
    last_check_time_ = now;
  }

  if (hit_count_ >= consecutive_hits_) {
    RCLCPP_WARN(
      node_->get_logger(), "IsStuckCondition: robot confirmed STUCK (cost=%u, hits=%d)", cost,
      hit_count_);
    // Reset so that next time it starts fresh
    hit_count_ = 0;
    first_tick_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace pb_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb_nav2_plugins::IsStuckCondition>("IsStuckCondition");
}
