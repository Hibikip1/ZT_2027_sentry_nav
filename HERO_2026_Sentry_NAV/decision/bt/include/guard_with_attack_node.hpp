#pragma once

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

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/global_target_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace BT;

/**
 * @brief 驻守追击行为节点
 *
 * 在指定的驻守时间内，以初始位置为锚点，在 move_radius 范围内
 * 追击最近的可见敌人。追击逻辑：
 *   1. 订阅 TrackerOutput 获取多目标数组，选择最近敌人
 *   2. 以敌人为圆心、engage_distance 为半径画圆
 *   3. 在圆周上从近到远搜索代价地图可行且在 move_radius 内的点
 *   4. 以固定间隔向 Nav2 发送导航目标
 *   5. 无敌人时回到初始驻守点
 *
 * 该节点设计为可被 ReactiveSequence 中的血量/时间条件打断：
 * onHalted() 会取消当前导航并干净退出。
 *
 * BT XML 用法：
 * @code{.xml}
 * <GuardWithAttack guard_duration="7.0"
 *                  move_radius="3.0"
 *                  engage_distance="2.5"
 *                  occ_threshold="50"
 *                  nav_update_interval="2.0"/>
 * @endcode
 */
class GuardWithAttackNode : public StatefulActionNode {
   public:
    GuardWithAttackNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config) {
        // 获取 ROS 节点
        auto node_result =
            config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;

        // 初始化 Nav2 action client
        nav_client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                node_, "navigate_to_pose");

        // 初始化 TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 读取话题名（构造时读取一次，避免每次 tick 解析）
        std::string tracker_topic = "/global_targets";
        std::string costmap_topic = "/global_costmap/costmap";
        std::string costmap_update_topic = "/global_costmap/costmap_updates";

        // 创建独立 callback group（不阻塞主线程）
        cb_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        cb_executor_.add_callback_group(cb_group_,
                                        node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        // 订阅 TrackerOutput (GlobalTargetArray)
        tracker_sub_ =
            node_->create_subscription<interfaces::msg::GlobalTargetArray>(
                tracker_topic, rclcpp::SensorDataQoS(),
                [this](
                    const interfaces::msg::GlobalTargetArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(tracker_mutex_);
                    latest_tracker_ = msg;
                    last_tracker_time_ = node_->now();
                },
                sub_opts);

        // 订阅 OccupancyGrid costmap (由于设置了 transient_local QoS，可以收到历史最后一帧)
        rclcpp::SubscriptionOptions costmap_sub_opts;
        costmap_sub_opts.callback_group = cb_group_;
        costmap_sub_opts.ignore_local_publications = false;

        costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(costmap_mutex_);
                latest_costmap_ = msg;
                RCLCPP_INFO(node_->get_logger(), "GuardWithAttack: 收到全量 costmap");
            },
            costmap_sub_opts);

        // 订阅 Costmap Update (增量更新)
        costmap_update_sub_ = node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            costmap_update_topic, rclcpp::SensorDataQoS(),
            [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(costmap_mutex_);
                if (!latest_costmap_) {
                    return; // 还没收到全量地图，忽略增量
                }
                
                // 校验更新边界是否合法
                if (msg->x < 0 || msg->y < 0 || 
                    msg->x + msg->width > latest_costmap_->info.width ||
                    msg->y + msg->height > latest_costmap_->info.height) {
                    RCLCPP_WARN(node_->get_logger(), "GuardWithAttack: Costmap update 边界越界，忽略");
                    return;
                }

                // 将增量数据拷贝到全量地图中
                size_t src_idx = 0;
                for (size_t y = 0; y < msg->height; ++y) {
                    size_t dst_idx = (msg->y + y) * latest_costmap_->info.width + msg->x;
                    for (size_t x = 0; x < msg->width; ++x) {
                        latest_costmap_->data[dst_idx + x] = msg->data[src_idx++];
                    }
                }
            },
            costmap_sub_opts);

        // 可视化发布器
        marker_pub_ =
            node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/guard_attack_vis", 1);

        // 启动 callback executor 线程
        cb_thread_ = std::thread([this]() { cb_executor_.spin(); });

        RCLCPP_INFO(node_->get_logger(),
                    "GuardWithAttack: 初始化完成, tracker='%s', costmap='%s'",
                    tracker_topic.c_str(), costmap_topic.c_str());
    }

    ~GuardWithAttackNode() override {
        cb_executor_.cancel();
        if (cb_thread_.joinable()) {
            cb_thread_.join();
        }
    }

    static PortsList providedPorts() {
        return {
            InputPort<double>("guard_duration", "驻守持续时间（秒）"),
            InputPort<double>(
                "sense_radius", 10.0,
                "感知/"
                "警戒半径，只追击在此范围内的敌人（米，以驻守锚点为中心）"),
            InputPort<double>("move_radius", 3.0,
                              "以初始位置为中心的最大活动半径（米）"),
            InputPort<double>("engage_distance", 2.5,
                              "与敌人应保持的交战距离（米）"),
            InputPort<int>("occ_threshold", 20, "OccupancyGrid cost 可行阈值"),
            InputPort<double>("nav_update_interval", 2.0,
                              "导航目标最小更新间隔（秒）"),
            InputPort<std::string>("tracker_topic", "/global_targets",
                                   "GlobalTargetArray 话题"),
            InputPort<std::string>("costmap_topic", "/global_costmap/costmap",
                                   "代价地图话题"),
        };
    }

    // =====================================================================
    //  onStart: 记录初始状态
    // =====================================================================
    NodeStatus onStart() override {
        // 读取参数
        if (!getInput("guard_duration", guard_duration_)) {
            RCLCPP_WARN(node_->get_logger(),
                        "GuardWithAttack: 使用默认驻守时间 7s");
            guard_duration_ = 7.0;
        }
        getInput("sense_radius", sense_radius_);
        getInput("move_radius", move_radius_);
        getInput("engage_distance", engage_distance_);
        getInput("occ_threshold", occ_threshold_);
        getInput("nav_update_interval", nav_update_interval_);

        // 重置状态
        goal_handle_.reset();
        nav_active_ = false;

        // 确保 Nav2 可用
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(),
                         "GuardWithAttack: Nav2 action server 不可用");
            return NodeStatus::FAILURE;
        }

        // 记录初始位姿（驻守锚点）
        if (!getCurrentPosition(guard_origin_x_, guard_origin_y_)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "GuardWithAttack: 无法获取初始位姿");
            return NodeStatus::FAILURE;
        }

        start_time_ = node_->now();
        last_nav_time_ =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

        RCLCPP_INFO(node_->get_logger(),
                    "GuardWithAttack: 开始驻守 %.1fs, 锚点=(%.2f, %.2f), "
                    "感知半径=%.1fm, 活动半径=%.1fm, 交战距离=%.1fm",
                    guard_duration_, guard_origin_x_, guard_origin_y_,
                    sense_radius_, move_radius_, engage_distance_);

        return NodeStatus::RUNNING;
    }

    // =====================================================================
    //  onRunning: 主循环
    // =====================================================================
    NodeStatus onRunning() override {
        // ---- 1. 驻守时间到期 → 成功退出 ----
        double elapsed = (node_->now() - start_time_).seconds();
        if (elapsed >= guard_duration_) {
            cancelNavigation();
            RCLCPP_INFO(node_->get_logger(),
                        "GuardWithAttack: 驻守时间到 (%.1fs), 正常退出",
                        elapsed);
            return NodeStatus::SUCCESS;
        }

        // ---- 2. 节流：是否到了重新计算/发送导航目标的时间 ----
        double since_last_nav = (node_->now() - last_nav_time_).seconds();
        if (since_last_nav < nav_update_interval_) {
            return NodeStatus::RUNNING;
        }

        // ---- 3. 获取自身位置 ----
        double self_x, self_y;
        if (!getCurrentPosition(self_x, self_y)) {
            return NodeStatus::RUNNING;  // TF 暂时不可用，跳过这轮
        }

        // ---- 4. 获取最近敌人 ----
        double enemy_x, enemy_y;
        bool has_enemy = findNearestEnemy(self_x, self_y, enemy_x, enemy_y);

        // ---- 5. 决策：有敌人 → 追击；无敌人 → 回锚点 ----
        double target_x, target_y;

        if (has_enemy) {
            // 计算候选追击点
            if (!computeAttackPoint(self_x, self_y, enemy_x, enemy_y, target_x,
                                    target_y)) {
                // 所有候选点不可行 → 回退策略：沿方向凑近
                if (!computeFallbackPoint(self_x, self_y, enemy_x, enemy_y,
                                          target_x, target_y)) {
                    // 完全无法移动，停在原地
                    return NodeStatus::RUNNING;
                }
            }
        } else {
            // 没有敌人：如果离锚点太远就回去
            double dist_to_origin =
                std::hypot(self_x - guard_origin_x_, self_y - guard_origin_y_);
            if (dist_to_origin < 0.3) {
                // 已经在锚点附近，不需要导航
                return NodeStatus::RUNNING;
            }
            target_x = guard_origin_x_;
            target_y = guard_origin_y_;
        }

        // ---- 6. 发送导航目标 ----
        sendNavigationGoal(target_x, target_y);
        last_nav_time_ = node_->now();

        return NodeStatus::RUNNING;
    }

    // =====================================================================
    //  onHalted: 被 ReactiveSequence 打断
    // =====================================================================
    void onHalted() override {
        cancelNavigation();
        RCLCPP_WARN(node_->get_logger(),
                    "GuardWithAttack: 被打断 (已驻守 %.1fs)",
                    (node_->now() - start_time_).seconds());
    }

   private:
    // =================================================================
    //  候选点算法
    // =================================================================

    /**
     * @brief 在敌人周围的 engage_distance 圆上搜索最佳追击点
     *
     * 算法：
     *   1. 圆周等分取候选点（间距 ~0.3m）
     *   2. 按到自身的距离排序（由近到远）
     *   3. 依次检查：(a) 在 move_radius 内 (b) costmap 可行
     *   4. 返回第一个通过的点
     */
    bool computeAttackPoint(double self_x, double self_y, double enemy_x,
                            double enemy_y, double& out_x, double& out_y) {
        // 获取 costmap 快照，只需在循环外读取一次
        nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap = latest_costmap_;
        }

        if (!costmap) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "GuardWithAttack: 尚未收到 costmap, 无法评估追击点");
            return false;
        }

        // 以自身到敌人的角度为起始角（优先搜索靠近自己的方向）
        double base_angle = std::atan2(self_y - enemy_y, self_x - enemy_x);
        const double vehicle_dim = 0.3;

        // 水波纹向外扩张搜索：最多扩大8次，每次增加 0.4m
        const int max_steps = 8;
        const double radius_step = 0.4;

        for (int step = 0; step < max_steps; ++step) {
            double current_radius = engage_distance_ + step * radius_step;
            const double circumference = 2.0 * M_PI * current_radius;
            const int num_candidates = std::max(
                8, static_cast<int>(std::ceil(circumference / vehicle_dim)));

            struct Candidate {
                double x, y, dist_to_self;
            };
            std::vector<Candidate> candidates;
            candidates.reserve(num_candidates);

            for (int i = 0; i < num_candidates; ++i) {
                double angle = base_angle + 2.0 * M_PI * i / num_candidates;
                double cx = enemy_x + current_radius * std::cos(angle);
                double cy = enemy_y + current_radius * std::sin(angle);
                double dist = std::hypot(cx - self_x, cy - self_y);
                candidates.push_back({cx, cy, dist});
            }

            // 在当前圆周上，按距离自身由近到远排序，优先选移动代价最小的点
            std::sort(candidates.begin(), candidates.end(),
                      [](const Candidate& a, const Candidate& b) {
                          return a.dist_to_self < b.dist_to_self;
                      });

            for (size_t i = 0; i < candidates.size(); ++i) {
                double cx = candidates[i].x;
                double cy = candidates[i].y;

                // 检查 move_radius 约束
                double dist_to_origin =
                    std::hypot(cx - guard_origin_x_, cy - guard_origin_y_);

                if (dist_to_origin > move_radius_) {
                    // 超出活动半径 → 截断到 move_radius 圆周上
                    double dir_x = cx - guard_origin_x_;
                    double dir_y = cy - guard_origin_y_;
                    double norm = std::hypot(dir_x, dir_y);
                    if (norm > 1e-6) {
                        cx = guard_origin_x_ + (dir_x / norm) * move_radius_;
                        cy = guard_origin_y_ + (dir_y / norm) * move_radius_;
                    } else {
                        continue;
                    }
                }

                // 检查 costmap 可行性
                if (!isCostmapFree(costmap, cx, cy)) {
                    continue;
                }

                // 找到可行点
                out_x = cx;
                out_y = cy;

                // 发布可视化
                publishVisualization(enemy_x, enemy_y, out_x, out_y);

                return true;
            }
        }

        return false;  // 所有圈无可行候选点
    }

    /**
     * @brief 回退策略：沿自身到敌人方向，在 move_radius 内寻找最远可行点
     *
     * 当圆周候选点全部不可行时调用。以 0.2m 步进从远到近搜索。
     */
    bool computeFallbackPoint(double self_x, double self_y, double enemy_x,
                              double enemy_y, double& out_x, double& out_y) {
        double dir_x = enemy_x - self_x;
        double dir_y = enemy_y - self_y;
        double dist = std::hypot(dir_x, dir_y);
        if (dist < 1e-6) return false;

        dir_x /= dist;
        dir_y /= dist;

        nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap = latest_costmap_;
        }

        if (!costmap) {
            return false;
        }

        // 从远到近搜索（先尝试尽可能接近敌人的位置）
        double max_advance = std::max(0.0, dist - 0.5);  // 不要撞上敌人
        const double step = 0.2;

        for (double d = max_advance; d >= 0; d -= step) {
            double cx = self_x + dir_x * d;
            double cy = self_y + dir_y * d;

            // 检查 move_radius
            double dist_to_origin =
                std::hypot(cx - guard_origin_x_, cy - guard_origin_y_);
            if (dist_to_origin > move_radius_) continue;

            // 检查 costmap
            if (!isCostmapFree(costmap, cx, cy)) continue;

            out_x = cx;
            out_y = cy;

            publishVisualization(enemy_x, enemy_y, out_x, out_y);
            return true;
        }

        return false;
    }

    // =================================================================
    //  Costmap 查询
    // =================================================================

    /**
     * @brief 检查世界坐标 (wx, wy) 在 costmap 上是否可行
     */
    bool isCostmapFree(const nav_msgs::msg::OccupancyGrid::SharedPtr& costmap,
                       double wx, double wy) const {
        const auto& info = costmap->info;
        int mx =
            static_cast<int>((wx - info.origin.position.x) / info.resolution);
        int my =
            static_cast<int>((wy - info.origin.position.y) / info.resolution);

        if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
            my >= static_cast<int>(info.height)) {
            return false;  // 超出地图范围视为不可行
        }

        int idx = my * info.width + mx;
        int8_t cost = costmap->data[idx];

        // OccupancyGrid: -1=unknown, 0=free, 100=occupied
        // 阈值可配置（默认 50）
        return (cost >= 0 && cost < occ_threshold_);
    }

    // =================================================================
    //  TF 查询
    // =================================================================

    bool getCurrentPosition(double& x, double& y) {
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link",
                                                         tf2::TimePointZero);
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "GuardWithAttack TF: %s", ex.what());
            return false;
        }
    }

    // =================================================================
    //  TrackerOutput 处理
    // =================================================================

    /**
     * @brief 从最新的 TrackerOutput 中选择距离自身最近的敌人
     *
     * 如果超过 3 秒无数据，视为无敌人。
     */
    bool findNearestEnemy(double self_x, double self_y, double& ex,
                          double& ey) {
        interfaces::msg::GlobalTargetArray::SharedPtr tracker;
        rclcpp::Time tracker_time;
        {
            std::lock_guard<std::mutex> lock(tracker_mutex_);
            tracker = latest_tracker_;
            tracker_time = last_tracker_time_;
        }

        if (!tracker || tracker->targets.empty()) {
            return false;
        }

        // 检查数据新鲜度（3 秒超时）
        if ((node_->now() - tracker_time).seconds() > 3.0) {
            return false;
        }

        double min_dist = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto& target : tracker->targets) {
            double tx = target.position.x;
            double ty = target.position.y;

            // 过滤掉不在驻守锚点感知半径内的敌人
            double dist_to_origin =
                std::hypot(tx - guard_origin_x_, ty - guard_origin_y_);
            if (dist_to_origin > sense_radius_) {
                continue;
            }

            double dist = std::hypot(tx - self_x, ty - self_y);
            if (dist < min_dist) {
                min_dist = dist;
                ex = tx;
                ey = ty;
                found = true;
            }
        }

        return found;
    }

    // =================================================================
    //  导航控制
    // =================================================================

    void sendNavigationGoal(double x, double y) {
        // 先取消旧的（如果有）
        cancelNavigation();

        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        goal.pose.pose.orientation = tf2::toMsg(q);

        auto send_options = rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
                std::lock_guard<std::mutex> lock(nav_mutex_);
                nav_active_ = false;
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "GuardWithAttack: 导航目标到达");
                } else {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "GuardWithAttack: 导航结束 (code=%d)",
                                 static_cast<int>(result.code));
                }
            };

        auto future = nav_client_->async_send_goal(goal, send_options);

        // 非阻塞地尝试获取 goal_handle（如果马上可用）
        if (future.wait_for(std::chrono::milliseconds(100)) ==
            std::future_status::ready) {
            std::lock_guard<std::mutex> lock(nav_mutex_);
            goal_handle_ = future.get();
            nav_active_ = (goal_handle_ != nullptr);
        } else {
            std::lock_guard<std::mutex> lock(nav_mutex_);
            nav_active_ = true;  // 乐观假设会被接受
        }

        RCLCPP_DEBUG(node_->get_logger(),
                     "GuardWithAttack: 导航目标 → (%.2f, %.2f)", x, y);
    }

    void cancelNavigation() {
        std::lock_guard<std::mutex> lock(nav_mutex_);
        if (goal_handle_) {
            try {
                auto status = goal_handle_->get_status();
                if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                    status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                    nav_client_->async_cancel_goal(goal_handle_);
                }
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(node_->get_logger(),
                             "GuardWithAttack: 取消导航异常 (已忽略): %s",
                             e.what());
            }
            goal_handle_.reset();
        }
        nav_active_ = false;
    }

    // =================================================================
    //  可视化
    // =================================================================

    void publishVisualization(double enemy_x, double enemy_y, double target_x,
                              double target_y) {
        visualization_msgs::msg::MarkerArray markers;

        // 敌人位置（红色球）
        visualization_msgs::msg::Marker enemy_marker;
        enemy_marker.header.frame_id = "map";
        enemy_marker.header.stamp = node_->now();
        enemy_marker.ns = "guard_attack";
        enemy_marker.id = 0;
        enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
        enemy_marker.action = visualization_msgs::msg::Marker::ADD;
        enemy_marker.pose.position.x = enemy_x;
        enemy_marker.pose.position.y = enemy_y;
        enemy_marker.pose.position.z = 0.1;
        enemy_marker.pose.orientation.w = 1.0;
        enemy_marker.scale.x = 0.2;
        enemy_marker.scale.y = 0.2;
        enemy_marker.scale.z = 0.2;
        enemy_marker.color.r = 1.0;
        enemy_marker.color.a = 0.8;
        enemy_marker.lifetime = rclcpp::Duration::from_seconds(3.0);
        markers.markers.push_back(enemy_marker);

        // 导航目标（绿色球）
        visualization_msgs::msg::Marker target_marker = enemy_marker;
        target_marker.id = 1;
        target_marker.pose.position.x = target_x;
        target_marker.pose.position.y = target_y;
        target_marker.color.r = 0.0;
        target_marker.color.g = 1.0;
        markers.markers.push_back(target_marker);

        // 驻守锚点（蓝色球）
        visualization_msgs::msg::Marker origin_marker = enemy_marker;
        origin_marker.id = 2;
        origin_marker.pose.position.x = guard_origin_x_;
        origin_marker.pose.position.y = guard_origin_y_;
        origin_marker.color.r = 0.0;
        origin_marker.color.b = 1.0;
        origin_marker.scale.x = 0.15;
        origin_marker.scale.y = 0.15;
        origin_marker.scale.z = 0.15;
        markers.markers.push_back(origin_marker);

        // 活动半径（蓝色圆柱）
        visualization_msgs::msg::Marker radius_marker;
        radius_marker.header.frame_id = "map";
        radius_marker.header.stamp = node_->now();
        radius_marker.ns = "guard_attack";
        radius_marker.id = 3;
        radius_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        radius_marker.action = visualization_msgs::msg::Marker::ADD;
        radius_marker.pose.position.x = guard_origin_x_;
        radius_marker.pose.position.y = guard_origin_y_;
        radius_marker.pose.position.z = 0.01;
        radius_marker.pose.orientation.w = 1.0;
        radius_marker.scale.x = move_radius_ * 2.0;
        radius_marker.scale.y = move_radius_ * 2.0;
        radius_marker.scale.z = 0.02;
        radius_marker.color.b = 1.0;
        radius_marker.color.a = 0.1;
        radius_marker.lifetime = rclcpp::Duration::from_seconds(3.0);
        markers.markers.push_back(radius_marker);

        marker_pub_->publish(markers);
    }

    // =================================================================
    //  成员变量
    // =================================================================

    rclcpp::Node::SharedPtr node_;

    // Nav2
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
        nav_client_;
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    std::mutex nav_mutex_;
    bool nav_active_{false};

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // TrackerOutput (GlobalTargetArray) 订阅
    rclcpp::Subscription<interfaces::msg::GlobalTargetArray>::SharedPtr
        tracker_sub_;
    std::mutex tracker_mutex_;
    interfaces::msg::GlobalTargetArray::SharedPtr latest_tracker_;
    rclcpp::Time last_tracker_time_;

    // Costmap 订阅
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_sub_;
    std::mutex costmap_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    // 可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;

    // Callback group + executor（后台线程处理订阅）
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor cb_executor_;
    std::thread cb_thread_;

    // 参数
    double guard_duration_{7.0};
    double sense_radius_{10.0};
    double move_radius_{3.0};
    double engage_distance_{2.5};
    int occ_threshold_{20};
    double nav_update_interval_{2.0};

    // 运行时状态
    double guard_origin_x_{0.0};
    double guard_origin_y_{0.0};
    rclcpp::Time start_time_;
    rclcpp::Time last_nav_time_;
};
