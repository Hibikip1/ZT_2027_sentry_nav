#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp/basic_types.h>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <std_msgs/msg/bool.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/gimbaldecision.hpp"
#include "interfaces/msg/target_switch.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace BT;
using json = nlohmann::json;

/**
 * @brief 驻守行为
 * @param 驻守时间[out]: guard_duration
 */
class GuardianNode : public StatefulActionNode {
   public:
    GuardianNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config) {
        // 获取ROS节点
        auto node_result =
            config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;
    }

    static PortsList providedPorts() {
        return {
            InputPort<double>("guard_duration", "驻守持续时间（秒）"),
        };
    }

    NodeStatus onStart() override {
        // 从黑板获取驻守时间（支持动态配置）
        if (!getInput("guard_duration", guard_duration_)) {
            RCLCPP_WARN(node_->get_logger(), "使用默认驻守时间4秒");
            guard_duration_ = 4.0;
        }

        start_time_ = node_->now();  // 记录驻守开始时间
        elapsed_time_ = 0;
        RCLCPP_INFO(node_->get_logger(), "开始驻守，持续时间%.1f秒",
                    guard_duration_);
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        elapsed_time_ = (node_->now() - start_time_).seconds();

        // 未达到驻守时间
        if (elapsed_time_ < guard_duration_) {
            RCLCPP_DEBUG(node_->get_logger(), "驻守中... 剩余%.1f秒",
                         guard_duration_ - elapsed_time_);
            return NodeStatus::RUNNING;
        }

        // 新增状态重置
        elapsed_time_ = 0;
        guard_duration_ = 0;
        return NodeStatus::SUCCESS;
    }

    void onHalted() override {
        // 记录被中断时的驻守时间
        RCLCPP_WARN(node_->get_logger(), "驻守被中断，已驻守%.1f秒",
                    elapsed_time_);
    }

   private:
    rclcpp::Node::SharedPtr node_;
    double guard_duration_;    // 驻守总时长
    rclcpp::Time start_time_;  // 开始时间戳
    double elapsed_time_;      // 已驻守时间
};

/**
 * @brief 追击行为
 * 导航到目标位置，但在距离目标还有2米时就认为导航成功
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param early_success_distance 欧氏距离阈值
 */
class AttackNavNode : public StatefulActionNode {
   public:
    AttackNavNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config) {
        // 获取ROS节点
        auto node_result =
            config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;

        // 初始化导航客户端
        action_client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                node_, "navigate_to_pose");

        // 初始化机器人状态订阅者
        robot_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "arrived_at_goal", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                goal_reached_ = msg->data;
            });

        // 初始化TF变换监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    static PortsList providedPorts() {
        return {InputPort<double>("x", "目标点X坐标"),
                InputPort<double>("y", "目标点Y坐标"),
                InputPort<double>("yaw", "目标朝向(rad, 可选, 默认0)"),
                InputPort<double>("early_success_distance",
                                  "提前成功的距离阈值，默认2.0米"),
                InputPort<std::string>("robot_frame",
                                       "机器人坐标系，默认base_link"),
                InputPort<std::string>("map_frame", "地图坐标系，默认map")};
    }

    NodeStatus onStart() override {
        goal_reached_ = false;
        goal_handle_.reset();
        future_goal_handle_ = {};
        early_success_ = false;

        // 确保动作服务器可用
        if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(), "导航服务不可用");
            return NodeStatus::FAILURE;
        }

        // 获取目标点
        if (!getInput("x", target_x_) || !getInput("y", target_y_)) {
            RCLCPP_ERROR(node_->get_logger(), "缺少必要的坐标输入");
            return NodeStatus::FAILURE;
        }

        // 获取可选参数
        if (!getInput("early_success_distance", early_success_distance_)) {
            early_success_distance_ = 2.0;  // 默认2米
        }

        // 获取可选目标朝向(过狗洞时正对洞口用)
        if (!getInput("yaw", target_yaw_)) {
            target_yaw_ = 0.0;  // 默认0
        }

        if (!getInput("robot_frame", robot_frame_)) {
            robot_frame_ = "base_link";  // 默认机器人坐标系
        }

        if (!getInput("map_frame", map_frame_)) {
            map_frame_ = "map";  // 默认地图坐标系
        }

        // 创建并发送导航目标
        auto goal = createGoal(target_x_, target_y_, target_yaw_);
        if (!goal) {
            return NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "追击目标点：(%.2f, %.2f)，提前结束距离：%.2f米", target_x_,
                    target_y_, early_success_distance_);

        future_goal_handle_ = action_client_->async_send_goal(*goal);
        // 等待服务器接受（比如最多等 1s），并拿到 goal_handle_
        if (rclcpp::spin_until_future_complete(node_, future_goal_handle_) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            goal_handle_ = future_goal_handle_.get();
        } else {
            RCLCPP_ERROR(node_->get_logger(), "目标请求未被接受");
            return NodeStatus::FAILURE;
        }
        // 开始执行
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        // 检查是否已到达目标（由底盘反馈）
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_reached_) {
                RCLCPP_INFO(node_->get_logger(), "底盘反馈已到达目标点");
                return NodeStatus::SUCCESS;
            }
        }

        // 计算当前与目标点的距离
        double current_distance = calculateDistanceToTarget();

        // 如果距离小于阈值，提前结束导航
        if (current_distance > 0 &&
            current_distance <= early_success_distance_) {
            if (!early_success_) {
                early_success_ = true;
                RCLCPP_INFO(node_->get_logger(),
                            "距离目标点仅剩%.2f米，提前结束导航",
                            current_distance);

                // 取消导航
                if (goal_handle_) {
                    auto future_cancel =
                        action_client_->async_cancel_goal(goal_handle_);
                    RCLCPP_INFO(node_->get_logger(), "取消导航成功");
                    if (rclcpp::spin_until_future_complete(node_,
                                                           future_cancel) !=
                        rclcpp::FutureReturnCode::SUCCESS) {
                        RCLCPP_ERROR(node_->get_logger(), "取消导航失败");
                    }
                }

                return NodeStatus::RUNNING;
            }
            return NodeStatus::RUNNING;
        }

        // 检查导航状态
        if (!goal_handle_) {
            if (future_goal_handle_.valid() &&
                future_goal_handle_.wait_for(std::chrono::seconds(0)) ==
                    std::future_status::ready) {
                goal_handle_ = future_goal_handle_.get();
            }
        }

        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // 取消导航
        if (goal_handle_) {
            auto future_cancel =
                action_client_->async_cancel_goal(goal_handle_);
            if (rclcpp::spin_until_future_complete(node_, future_cancel) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node_->get_logger(), "取消导航失败");
            }
        }
    }

   private:
    std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> createGoal(
        double x, double y, double yaw) {
        auto goal = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();
        goal->pose.header.frame_id = map_frame_;
        goal->pose.header.stamp = node_->now();
        goal->pose.pose.position.x = x;
        goal->pose.pose.position.y = y;
        goal->pose.pose.position.z = 0.0;

        // 设置朝向(过狗洞/任务区域时用指定 yaw 正对目标)
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal->pose.pose.orientation = tf2::toMsg(q);

        return goal;
    }

    // 计算当前位置到目标点的距离
    double calculateDistanceToTarget() {
        try {
            // 获取机器人在地图坐标系中的当前位置
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(map_frame_, robot_frame_,
                                            tf2::TimePointZero);

            // 计算欧几里得距离
            double dx = transform.transform.translation.x - target_x_;
            double dy = transform.transform.translation.y - target_y_;
            double distance = std::sqrt(dx * dx + dy * dy);

            RCLCPP_DEBUG(node_->get_logger(), "当前距离目标点：%.2f米",
                         distance);
            return distance;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "无法获取TF变换: %s", ex.what());
            return -1.0;  // 返回负值表示计算失败
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
        action_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_future<typename rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr>
        future_goal_handle_;
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    double target_x_;
    double target_y_;
    double target_yaw_{0.0};
    double early_success_distance_;
    std::string robot_frame_;
    std::string map_frame_;

    bool goal_reached_ = false;
    bool early_success_ = false;
    std::mutex mutex_;
};

/**
 * @brief 自瞄控制节点 更新黑板参数便于定时器取用
 */
class AutoAimManagerNode : public ConditionNode {
   public:
    AutoAimManagerNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {
        auto node_result =
            config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;
    }

    static PortsList providedPorts() {
        return {
            InputPort<std::string>("targets", "目标优先级数组（JSON 格式）"),
            };
    }

    BT::NodeStatus tick() override {
        uint8_t is_auto_aim, auto_aim_mode, is_gimbal_patrol, is_reached;
        float yaw_degrees;
        std::string targets_str;
        std::vector<interfaces::msg::TargetSwitch> targets;

        // 从端口获取所有输入并写入黑板
        if (getInput("targets", targets_str)) {
            try {
                // 解析 JSON 字符串
                json j = json::parse(targets_str);
               
                // 将 JSON 数据转换为 Target 类型，并填充到 targets 数组
                for (auto& el : j) {
                    interfaces::msg::TargetSwitch target;
                    target.type = el["type"].get<std::string>();
                    target.priority = el["priority"].get<uint8_t>();
                    target.distance = el["distance"].get<float>();
                    targets.push_back(target);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing targets: %s",
                             e.what());
                return BT::NodeStatus::FAILURE;
            }

            // 将 targets 写入黑板
            config().blackboard->set("targets", targets);
            targets.clear();
        }

        return NodeStatus::SUCCESS;
    }

   private:
    rclcpp::Node::SharedPtr node_;
};

/**
 * @brief 清空标志位节点
 */
class ClearFlagNode : public BT::SyncActionNode {
   public:
    ClearFlagNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("flag_name", "黑板上的标志位名字"),
            BT::InputPort<int>("expected_value", "期望改动的数据（如0）")
        };
    }

    BT::NodeStatus tick() override {
        std::string flag_name;
        int expected_value;

        if (!getInput("flag_name", flag_name)) {
            throw BT::RuntimeError("missing required input [flag_name]");
        }
        if (!getInput("expected_value", expected_value)) {
            throw BT::RuntimeError("missing required input [expected_value]");
        }

        config().blackboard->set(flag_name, expected_value);
        return BT::NodeStatus::SUCCESS;
    }
};
