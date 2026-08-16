#pragma once
#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "bt_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_interfaces/msg/robot.hpp"
#include "serial_interfaces/msg/tovision.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace BT;

/**
 * @brief 状态控制节点，用于调控标志位
 */
class CruiserStateManagerNode : public ConditionNode {
   public:
    CruiserStateManagerNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<int>("cruise_state_in", "当前巡航状态"),
                InputPort<int>("total_points", "总导航点数"),
                OutputPort<int>("cruise_state_out", "更新后的巡航状态")};
    }

    NodeStatus tick() override {
        int current_state, total_points;

        // 获取输入参数
        if (!getInput("cruise_state_in", current_state) ||
            !getInput("total_points", total_points)) {
            return NodeStatus::FAILURE;
        }

        // 状态更新逻辑
        int new_state = (current_state + 1) % total_points;
        setOutput("cruise_state_out", new_state);

        return NodeStatus::SUCCESS;
    }
};

/**
 * @brief 导航节点，保留导航和判断是否到达（依赖导航句柄）的功能
 * @param 导航点坐标 x，y
 */
class NavNode : public StatefulActionNode {
   public:
    NavNode(const std::string& name, const NodeConfig& config)
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

        // 初始化到达状态发布者
        arrival_pub_ =
            node_->create_publisher<std_msgs::msg::Bool>("arrived_at_goal", 10);
    }

    static PortsList providedPorts() {
        return {InputPort<double>("x", "目标点X坐标"),
                InputPort<double>("y", "目标点Y坐标")};
    }

    NodeStatus onStart() override {
        // 重置所有状态
        goal_handle_.reset();
        future_goal_handle_ = {};
        goal_result_future_ = {};
        navigation_result_ = rclcpp_action::ResultCode::UNKNOWN;
        result_received_ = false;

        // 确保动作服务器可用
        if (!action_client_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available");
            return NodeStatus::FAILURE;
        }

        // 获取目标点
        if (!getInput("x", patrol_points_.first) ||
            !getInput("y", patrol_points_.second)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input");
            return NodeStatus::FAILURE;
        }

        // 创建并发送导航目标
        auto goal = createGoal(patrol_points_.first, patrol_points_.second);
        if (!goal) {
            RCLCPP_ERROR(node_->get_logger(), "无法创建目标点");
            return NodeStatus::FAILURE;
        }

        // 设置目标选项，包括结果回调
        auto send_goal_options = rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
                std::lock_guard<std::mutex> lock(mutex_);
                navigation_result_ = result.code;
                result_received_ = true;
            };

        // 异步发送目标
        future_goal_handle_ =
            action_client_->async_send_goal(*goal, send_goal_options);

        // 发布导航开始状态
        auto arrived_msg = std_msgs::msg::Bool();
        arrived_msg.data = false;
        arrival_pub_->publish(arrived_msg);
        RCLCPP_INFO(node_->get_logger(), "开始导航到目标点：(%.2f, %.2f)",
                    patrol_points_.first, patrol_points_.second);

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        // 首先检查是否获得了goal handle
        if (!goal_handle_) {
            if (future_goal_handle_.valid() &&
                future_goal_handle_.wait_for(0s) == std::future_status::ready) {
                try {
                    goal_handle_ = future_goal_handle_.get();
                    if (!goal_handle_) {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "Goal was rejected by server");
                        return NodeStatus::FAILURE;
                    }
                    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Exception in goal handle: %s", e.what());
                    return NodeStatus::FAILURE;
                }
            } else {
                // 还在等待goal handle，继续运行
                return NodeStatus::RUNNING;
            }
        }

        // 检查导航结果
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (result_received_) {
                switch (navigation_result_) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node_->get_logger(), "成功到达目标点");
                        publishArrivalStatus(true);
                        return NodeStatus::SUCCESS;

                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(node_->get_logger(), "导航被中止");
                        publishArrivalStatus(false);
                        return NodeStatus::FAILURE;

                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(node_->get_logger(), "导航被取消");
                        publishArrivalStatus(false);
                        return NodeStatus::FAILURE;

                    default:
                        RCLCPP_ERROR(node_->get_logger(), "导航出现未知错误");
                        publishArrivalStatus(false);
                        return NodeStatus::FAILURE;
                }
            }
        }

        // 可选：检查goal handle状态作为额外验证
        if (goal_handle_) {
            try {
                auto status = goal_handle_->get_status();
                if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
                    status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
                    RCLCPP_WARN(node_->get_logger(), "Goal handle状态异常: %d",
                                status);
                    publishArrivalStatus(false);
                    return NodeStatus::FAILURE;
                }
            } catch (const std::exception& e) {
                // 若底层抛出未知句柄异常，说明目标状态早已被服务端结束和清理
                RCLCPP_WARN(node_->get_logger(),
                            "尝试获取异常的目标状态(直接当失败处理): %s",
                            e.what());
                publishArrivalStatus(false);
                return NodeStatus::FAILURE;
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "还没到");
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // 取消当前导航任务
        if (goal_handle_) {
            try {
                auto status = goal_handle_->get_status();
                // 只有当目标处于接收或执行状态时才尝试取消
                if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                    status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                    RCLCPP_INFO(node_->get_logger(), "取消导航任务");
                    auto future_cancel =
                        action_client_->async_cancel_goal(goal_handle_);
                    // 不等待取消结果，避免阻塞
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(),
                            "Halt取消目标时忽略异常(目标可能已被清理): %s",
                            e.what());
            }
        }

        // 重置状态
        {
            std::lock_guard<std::mutex> lock(mutex_);
            result_received_ = false;
            navigation_result_ = rclcpp_action::ResultCode::UNKNOWN;
        }

        publishArrivalStatus(false);
    }

   private:
    std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> createGoal(
        double x, double y) {
        auto goal = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();
        goal->pose.header.frame_id = "map";
        goal->pose.header.stamp = node_->now();
        goal->pose.pose.position.x = x;
        goal->pose.pose.position.y = y;
        goal->pose.pose.position.z = 0.0;

        // 设置朝向（默认为0）
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        goal->pose.pose.orientation = tf2::toMsg(q);

        return goal;
    }

    void publishArrivalStatus(bool arrived) {
        auto arrived_msg = std_msgs::msg::Bool();
        arrived_msg.data = arrived;
        arrival_pub_->publish(arrived_msg);
    }

    // ROS相关成员
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
        action_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrival_pub_;

    // 导航状态成员
    std::pair<double, double> patrol_points_;
    std::shared_future<typename rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr>
        future_goal_handle_;
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

    // 使用结果回调机制替代状态订阅
    std::shared_future<typename rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::WrappedResult>
        goal_result_future_;
    rclcpp_action::ResultCode navigation_result_ =
        rclcpp_action::ResultCode::UNKNOWN;
    bool result_received_ = false;

    // 线程安全
    std::mutex mutex_;
};