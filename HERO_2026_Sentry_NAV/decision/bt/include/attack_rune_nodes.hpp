#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

// 包含刚才自定义的NavOutput消息
#include "interfaces/msg/nav_output.hpp"

using namespace BT;

/**
 * @brief 发送目标TF及云台模式的单次同步动作节点
 */
class PublishNavOutputNode : public SyncActionNode {
   public:
    PublishNavOutputNode(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 创建独立的 TF 缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建发布者
        publisher_ = node_->create_publisher<interfaces::msg::NavOutput>(
            "/carrot_pose", rclcpp::SensorDataQoS());
    }

    static PortsList providedPorts() {
        return {InputPort<std::string>("target_tf_name",
                                       "Target TF coordinate frame name"),
                InputPort<int>("mode",
                               "0: Nav, 1: Outpost, 2: Small Energy, 3: Large "
                               "Energy, 4: Disable")};
    }

    NodeStatus tick() override {
        std::string target_tf_name;
        int mode;
        if (!getInput("target_tf_name", target_tf_name) ||
            !getInput("mode", mode)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "PublishNavOutputNode: Missing required inputs!");
            return NodeStatus::FAILURE;
        }

        try {
            // 获取 target_tf_name 在 base_link 下的坐标
            auto transform_stamped = tf_buffer_->lookupTransform(
                "base_link", target_tf_name, tf2::TimePointZero);

            interfaces::msg::NavOutput msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = "base_link";
            msg.mode = mode;

            msg.point.header = msg.header;
            msg.point.pose.position.x =
                transform_stamped.transform.translation.x;
            msg.point.pose.position.y =
                transform_stamped.transform.translation.y;
            msg.point.pose.position.z =
                transform_stamped.transform.translation.z;
            msg.point.pose.orientation = transform_stamped.transform.rotation;

            publisher_->publish(msg);
            RCLCPP_INFO(node_->get_logger(),
                        "Published NavOutput: tf=%s, mode=%d",
                        target_tf_name.c_str(), mode);

            return NodeStatus::SUCCESS;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(),
                        "PublishNavOutputNode: TF transform failed: %s",
                        ex.what());
            return NodeStatus::FAILURE;
        }
    }

   private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<interfaces::msg::NavOutput>::SharedPtr publisher_;
};

/**
 * @brief 能量机关击打状态机节点（含有20s超时与状态轮询的持续性动作节点）
 */
class RuneStateMachineNode : public StatefulActionNode {
   public:
    RuneStateMachineNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        blackboard_ = config.blackboard;

        // 发布串口触发指令的话题
        activate_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "RuneActivateCommand", 10);

        // 发布自瞄模式的话题
        nav_output_pub_ = node_->create_publisher<interfaces::msg::NavOutput>(
            "nav_output", 10);
    }

    static PortsList providedPorts() {
        return {
            InputPort<std::string>("flag_name",
                                   "Blackboard flag name to set to 1 on entry"),
            InputPort<std::string>("status_key",
                                   "Key on blackboard to monitor setup (e.g., "
                                   "small_energy_state)"),
            InputPort<int>("target_state",
                           "State value representing activated (default 1)")};
    }

    NodeStatus onStart() override {
        // 进入打符状态：记录时间
        start_time_ = node_->now();

        // 设置黑板上的标志位
        std::string flag_name;
        if (getInput("flag_name", flag_name)) {
            blackboard_->set<int>(flag_name, 1);
        }

        // 发布开始打符请求 (activate = true)
        publishActivateCommand(true);
        RCLCPP_INFO(node_->get_logger(),
                    "RuneStateMachineNode Started, triggered activate=true, "
                    "set flag: %s",
                    flag_name.c_str());

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto now = node_->now();
        auto duration = (now - start_time_).seconds();

        // 获取需要读取的黑板键值名称与目标脱出状态
        std::string status_key;
        int target_state = 1;
        if (!getInput("status_key", status_key)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "RuneStateMachineNode: missing status_key!");
            return NodeStatus::FAILURE;
        }
        getInput("target_state", target_state);

        int current_rune_status = 0;
        blackboard_->get(status_key, current_rune_status);

        // 当状态达到目标激活状态
        if (current_rune_status == target_state) {
            RCLCPP_INFO(node_->get_logger(),
                        "Rune (%s) Activated successfully!",
                        status_key.c_str());
            publishActivateCommand(false);
            // publishDisableAutoAim();
            return NodeStatus::SUCCESS;
        }

        // 超过 20s 未达到状态3，即使在激活中也算失败退出
        if (duration > 20.0) {
            RCLCPP_WARN(node_->get_logger(), "Rune Activation TIMEOUT (20s)!");
            publishActivateCommand(false);
            // publishDisableAutoAim();
            return NodeStatus::FAILURE;
        }

        // 继续循环 (状态还在 1未激活 或 2正在激活)
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // 当节点被如掉血逃跑等高级行为打断时，安全地发出取消指令
        RCLCPP_INFO(node_->get_logger(),
                    "RuneStateMachineNode Halted (interrupted)");
        publishActivateCommand(false);
        // publishDisableAutoAim();
    }

   private:
    void publishActivateCommand(bool request_activate) {
        std_msgs::msg::Bool msg;
        msg.data = request_activate;
        activate_pub_->publish(msg);
    }

    void publishDisableAutoAim() {
        interfaces::msg::NavOutput msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "base_link";
        msg.mode = 4;  // Disable mode
        msg.point.header = msg.header;
        msg.point.pose.position.x = 0.0;
        msg.point.pose.position.y = 0.0;
        msg.point.pose.position.z = 0.0;
        // orientation 默认为全0或四元数无旋转状态
        msg.point.pose.orientation.w = 1.0;
        msg.point.pose.orientation.x = 0.0;
        msg.point.pose.orientation.y = 0.0;
        msg.point.pose.orientation.z = 0.0;
        nav_output_pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Time start_time_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr activate_pub_;
    rclcpp::Publisher<interfaces::msg::NavOutput>::SharedPtr nav_output_pub_;
};
