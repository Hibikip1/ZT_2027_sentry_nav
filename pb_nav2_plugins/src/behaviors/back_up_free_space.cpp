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

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <cmath>

namespace pb_nav2_behaviors
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "max_speed", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "safe_distance_threshold", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, "ramp_duration", rclcpp::ParameterValue(1.0));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("max_speed", max_speed_);
  node->get_parameter("safe_distance_threshold", safe_distance_threshold_);
  node->get_parameter("ramp_duration", ramp_duration_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
}

void BackUpFreeSpace::onCleanup() { costmap_client_.reset(); }

// ---------------------------------------------------------------------------
// onRun – compute escape direction from the ESDF gradient
// ---------------------------------------------------------------------------

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  // ---- 1. Wait for the GetCostmap service ----
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the costmap service.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "Costmap service not available, waiting...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto future = costmap_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Timed out waiting for costmap service response.");
    return nav2_behaviors::Status::FAILED;
  }

  auto response = future.get();
  if (!response) {
    RCLCPP_ERROR(logger_, "Received null response from costmap service.");
    return nav2_behaviors::Status::FAILED;
  }
  const auto & costmap_msg = response->map;

  // ---- 2. Build the local ESDF from the costmap message ----
  if (!esdf_.updateFromCostmapMsg(costmap_msg)) {
    RCLCPP_ERROR(logger_, "Failed to build local ESDF from costmap.");
    return nav2_behaviors::Status::FAILED;
  }

  // ---- 3. Obtain the current robot pose ----
  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  const double rx = initial_pose_.pose.position.x;
  const double ry = initial_pose_.pose.position.y;

  // ---- 4. Check if the robot is already in free space ----
  if (esdf_.isInside(rx, ry) && esdf_.getDistance(rx, ry) >= safe_distance_threshold_) {
    RCLCPP_INFO(
      logger_, "Robot is already in free space (d=%.3f m). Nothing to do.",
      esdf_.getDistance(rx, ry));
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // ---- 5. Compute escape direction from the ESDF gradient ----
  double gx = 0.0, gy = 0.0;
  esdf_.getGradient(rx, ry, gx, gy);

  const double grad_norm = std::hypot(gx, gy);
  if (grad_norm < 1e-6) {
    RCLCPP_WARN(logger_, "ESDF gradient is near zero – cannot determine escape direction.");
    return nav2_behaviors::Status::FAILED;
  }

  escape_dir_x_ = gx / grad_norm;
  escape_dir_y_ = gy / grad_norm;

  // ---- 6. Set up timing ----
  command_x_ = command->target.x;  // maximum travel distance
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;
  start_time_ = clock_->now();

  RCLCPP_WARN(
    logger_, "BackUpFreeSpace: escaping %.2f m towards free space (dir=[%.2f, %.2f], d=%.3f m)",
    command_x_, escape_dir_x_, escape_dir_y_, esdf_.getDistance(rx, ry));

  return nav2_behaviors::Status::SUCCEEDED;
}

// ---------------------------------------------------------------------------
// onCycleUpdate – smooth ramp-up & distance-based exit
// ---------------------------------------------------------------------------

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  // ---- 1. Timeout check ----
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "BackUpFreeSpace: exceeded time allowance – aborting.");
    return nav2_behaviors::Status::FAILED;
  }

  // ---- 2. Current pose ----
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  const double rx = current_pose.pose.position.x;
  const double ry = current_pose.pose.position.y;

  // ---- 3. ESDF distance check – success condition ----
  if (esdf_.isInside(rx, ry) && esdf_.getDistance(rx, ry) >= safe_distance_threshold_) {
    stopRobot();
    RCLCPP_INFO(
      logger_, "BackUpFreeSpace: reached free space (d=%.3f m). Done.", esdf_.getDistance(rx, ry));
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // ---- 4. Maximum travel distance check ----
  const double diff_x = initial_pose_.pose.position.x - rx;
  const double diff_y = initial_pose_.pose.position.y - ry;
  const double distance = std::hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    RCLCPP_WARN(logger_, "BackUpFreeSpace: reached max travel distance – stopping.");
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // ---- 5. Smooth ramp-up speed ----
  const double elapsed = (clock_->now() - start_time_).seconds();
  const double alpha = std::clamp(elapsed / ramp_duration_, 0.0, 1.0);
  const double speed = MIN_SPEED + (max_speed_ - MIN_SPEED) * alpha;

  // ---- 6. Publish velocity command ----
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = escape_dir_x_ * speed;
  cmd_vel->linear.y = escape_dir_y_ * speed;
  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
