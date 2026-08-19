// 区域相关行为树节点
// 1. CheckZoneNode: 订阅 zone_info, 判断机器人是否在指定区域内
//    - 输入 zone_name: 目标区域名
//    - 黑板输出: in_zone(bool), zone_nav_yaw(double), zone_spin(bool)
// 2. SetSpinNode: 控制小陀螺
//    - 输入 spin(bool): true=开启小陀螺, false=关闭
//    - 通过 serial_interfaces/Tovision 的 expect_spin_degree 控制

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/action_node.h"

#include <example_interfaces/msg/float32.hpp>
#include <interfaces/msg/zone_info.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bt_nodes {

// ============ CheckZoneNode: 是否在指定区域 ============
class CheckZoneNode : public BT::ConditionNode {
   public:
    CheckZoneNode(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {
        auto node_result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw BT::RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;

        zone_sub_ = node_->create_subscription<interfaces::msg::ZoneInfo>(
            "zone_info", 10,
            [this](const interfaces::msg::ZoneInfo::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                current_zone_ = msg->zone_name;
                in_zone_ = msg->in_zone;
                nav_yaw_ = msg->nav_yaw;
                spin_enabled_ = msg->spin_enabled;
            });
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("zone_name", "要检查的区域名"),
            BT::OutputPort<bool>("in_zone", "是否在区域内"),
            BT::OutputPort<double>("zone_nav_yaw", "区域导航朝向"),
            BT::OutputPort<bool>("zone_spin", "区域是否允许陀螺"),
        };
    }

    BT::NodeStatus tick() override {
        std::string target_zone;
        if (!getInput("zone_name", target_zone)) {
            RCLCPP_WARN(node_->get_logger(), "CheckZone: 缺少 zone_name 输入");
            return BT::NodeStatus::FAILURE;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        bool matched = in_zone_ && (current_zone_ == target_zone);

        setOutput("in_zone", matched);
        setOutput("zone_nav_yaw", nav_yaw_);
        setOutput("zone_spin", spin_enabled_);

        return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<interfaces::msg::ZoneInfo>::SharedPtr zone_sub_;
    std::mutex mutex_;
    std::string current_zone_{""};
    bool in_zone_{false};
    double nav_yaw_{0.0};
    bool spin_enabled_{true};
};

// ============ SetSpinNode: 控制小陀螺 ============
// 仿真: 发布 cmd_spin(Float32 角速度) -> fake_vel_transform 叠加到底盘角速度
//   spin=true  -> 恒定角速度(默认 3.14 rad/s)
//   spin=false -> 0 (停转)
class SetSpinNode : public BT::SyncActionNode {
   public:
    SetSpinNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {
        auto node_result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw BT::RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;

        spin_pub_ = node_->create_publisher<example_interfaces::msg::Float32>(
            "cmd_spin", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("spin", "true=开陀螺, false=关陀螺"),
            BT::InputPort<double>("spin_speed", "陀螺角速度(rad/s, 默认3.14)"),
        };
    }

    BT::NodeStatus tick() override {
        bool spin = true;
        double spin_speed = 3.14;
        getInput("spin", spin);
        getInput("spin_speed", spin_speed);

        example_interfaces::msg::Float32 msg;
        msg.data = spin ? static_cast<float>(spin_speed) : 0.0f;
        spin_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "SetSpin: %s (%.2f rad/s)",
                    spin ? "开启" : "关闭", msg.data);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr spin_pub_;
};

}  // namespace bt_nodes
