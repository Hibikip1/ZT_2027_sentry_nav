#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_interfaces/msg/robot.hpp"
#include "serial_interfaces/msg/tovision.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace BT;
using namespace std::literals;

// 检查值节点
class CheckValueNode : public ConditionNode {
   public:
    CheckValueNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {
        // 从端口获取比较运算符，默认使用等于
        if (!getInput("operator", compare_operator_)) {
            compare_operator_ = "eq";
        }
    }

    static PortsList providedPorts() {
        return {InputPort<int>("value", "要检查的整数值"),
                InputPort<int>("expected", "期望的整数值"),
                InputPort<std::string>("operator",
                                       "比较运算符: eq, ne, gt, lt, ge, le")};
    }

    NodeStatus tick() override {
        // 获取输入值
        int value;
        int expected;

        if (!getInput("value", value) || !getInput("expected", expected)) {
            std::cout << "检查错误: 缺少输入值" << std::endl;
            return NodeStatus::FAILURE;
        }

        // 执行比较操作
        bool result = compareValues(value, expected);
        return result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

   private:
    bool compareValues(int value, int expected) {
        std::cout << "执行比较操作 [" << value << " " << compare_operator_
                  << " " << expected << "]" << std::endl;

        if (compare_operator_ == "eq") return value == expected;
        if (compare_operator_ == "ne") return value != expected;
        if (compare_operator_ == "gt") return value > expected;
        if (compare_operator_ == "lt") return value < expected;
        if (compare_operator_ == "ge") return value >= expected;
        if (compare_operator_ == "le") return value <= expected;

        throw RuntimeError("不支持的运算符: " + compare_operator_);
    }

    std::string compare_operator_;
};

class DoubleCheckValueNode : public ConditionNode {
   public:
    DoubleCheckValueNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {
        // 构造时不改变黑板变量（交由黑板持久化）
    }

    static PortsList providedPorts() {
        return {InputPort<int>("value", "如果使用此值，作为输入进行比对"),
                InputPort<int>("trigger_value", "触发激活此行为的阈值"),
                InputPort<std::string>(
                    "trigger_operator",
                    "触发激活逻辑操作符 (eq, gt, lt, ge, le, 等)"),
                InputPort<int>("exit_value", "退出脱离此行为的阈值"),
                InputPort<std::string>("exit_operator", "退出脱离逻辑操作符"),
                InputPort<std::string>("flag_name",
                                       "用于标志已经触发了本阶段的黑板变量名"),
                InputPort<std::string>("blackboard_key",
                                       "直接检查黑板上该值的选项(可选)")};
    }

    NodeStatus tick() override {
        int val;
        std::string blackboard_key;
        if (getInput("blackboard_key", blackboard_key) &&
            !blackboard_key.empty()) {
            if (!config().blackboard->get(blackboard_key, val)) {
                return NodeStatus::FAILURE;
            }
        } else if (!getInput("value", val)) {
            return NodeStatus::FAILURE;
        }

        int trigger_val, exit_val;
        std::string trigger_op_str, exit_op_str, flag_name;

        if (!getInput("trigger_value", trigger_val) ||
            !getInput("exit_value", exit_val) ||
            !getInput("trigger_operator", trigger_op_str) ||
            !getInput("exit_operator", exit_op_str) ||
            !getInput("flag_name", flag_name)) {
            return NodeStatus::FAILURE;
        }

        auto compare = [](int val, int expected, const std::string& op) {
            if (op == "eq") return val == expected;
            if (op == "gt") return val > expected;
            if (op == "lt") return val < expected;
            if (op == "ge") return val >= expected;
            if (op == "le") return val <= expected;
            if (op == "ne") return val != expected;
            return false;
        };

        // 从黑板拉取历史触发记录：如果没有则设为 0
        int is_triggered = 0;
        config().blackboard->get(flag_name, is_triggered);

        if (is_triggered == 0) {
            // == 尚未触发该阶段事件，监听是否达到触发阈门 ==
            if (compare(val, trigger_val, trigger_op_str)) {
                // 已满足触发条件！开启急救/状态保持，并设置上锁标志位
                config().blackboard->set(flag_name, 1);
                return NodeStatus::SUCCESS;
            }
            // 未满足触发条件，保持不阻挡其他Fallback分支
            return NodeStatus::FAILURE;
        } else {
            // == 当前处于已触发激活的状态，必须紧盯着退出条件是否达标 ==
            if (compare(val, exit_val, exit_op_str)) {
                // 已满足打断条件：脱离危险/本阶段结束，降下标志位并放权
                config().blackboard->set(flag_name, 0);
                return NodeStatus::FAILURE;
            }
            // 还没从状态熬出头（比如血量才充到 250，还没大于 300 ！）
            // 那么返回SUCCESS以拦截Fallback树继续往下探！保持主导权
            return NodeStatus::SUCCESS;
        }
    }
};

// 打印日志节点
class LogMessageNode : public SyncActionNode {
   public:
    LogMessageNode(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {
        // 从blackboard获取ROS2节点
        auto result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (result) {
            node_ = result;
        }
    }

    static PortsList providedPorts() {
        return {InputPort<std::string>("message", "要记录的日志消息"),
                InputPort<std::string>("level", "日志级别 (info/warn/error)",
                                       "info")};
    }

    NodeStatus tick() override {
        std::string message;
        std::string level = "info";

        // 获取输入参数
        if (!getInput("message", message)) {
            if (node_) {
                RCLCPP_ERROR(node_->get_logger(),
                             "LogMessageNode 缺少message参数");
            }
            return NodeStatus::FAILURE;
        }
        getInput("level", level);  // 可选参数，默认info

        // 替换消息中的变量（格式：{var_name}）
        message = replaceBlackboardVars(message);

        // 根据级别输出日志
        if (node_) {
            if (level == "warn") {
                RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
            } else if (level == "error") {
                RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
            } else {
                RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
            }
        } else {
            // 备用控制台输出
            std::cout << "[LOG][" << level << "] " << message << std::endl;
        }

        return NodeStatus::SUCCESS;
    }

   private:
    std::string replaceBlackboardVars(const std::string& input) {
        std::string output = input;
        size_t start_pos = 0;

        while ((start_pos = output.find("{", start_pos)) != std::string::npos) {
            size_t end_pos = output.find("}", start_pos);
            if (end_pos == std::string::npos) break;

            std::string var_name =
                output.substr(start_pos + 1, end_pos - start_pos - 1);
            std::string var_value;

            if (auto entry = config().blackboard->getAny(var_name)) {
                var_value = BT::toStr(*entry);
            } else {
                var_value = "<undefined>";
            }

            output.replace(start_pos, end_pos - start_pos + 1, var_value);
            start_pos += var_value.length();
        }
        return output;
    }

    rclcpp::Node::SharedPtr node_;
};

class CheckCountdownNode : public ConditionNode {
   public:
    CheckCountdownNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<int>("remaining_time", "剩余时间（秒）"),
                InputPort<int>("check_start", "检测区间开始时间（秒）"),
                InputPort<int>("check_end", "检测区间结束时间（秒）")};
    }

    NodeStatus tick() override {
        int remaining, start, end;

        if (!getInput("remaining_time", remaining) ||
            !getInput("check_start", start) || !getInput("check_end", end)) {
            return NodeStatus::FAILURE;
        }

        // 自动处理时间区间顺序
        const int min = std::min(start, end);
        const int max = std::max(start, end);

        return (remaining >= min && remaining <= max) ? NodeStatus::SUCCESS
                                                      : NodeStatus::FAILURE;
    }
};

class ControlAutoAim : public SyncActionNode {
   public:
    ControlAutoAim(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {
        // 从blackboard获取ROS2节点
        auto result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = result;

        // 初始化订阅者
        std::string topic_name;
        if (!getInput("topic", topic_name)) {
            topic_name = "refree_msg";  // 默认话题名
        }
        publisher_ = node_->create_publisher<serial_interfaces::msg::Tovision>(
            "/tovision", 10);

        RCLCPP_INFO(rclcpp::get_logger("refree_node"), "裁判订阅节点启动");
    }

    static PortsList providedPorts() {
        return {// 目标参数
                InputPort<int>("bullet_17mm", "允许17mm发弹量"),
                InputPort<int>("competition_stage", "比赛状态")

        };
    }

    NodeStatus tick() override {
        int allow_bullet_17mm = 0;
        int competition_stage = 0;

        getInput("allow_bullet_17mm", allow_bullet_17mm);
        getInput("competition_stage", competition_stage);

        std::cout << "allow_bullet_17mm:" << allow_bullet_17mm << std::endl;
        if (allow_bullet_17mm > 0 && competition_stage == 4) {
            auto message = serial_interfaces::msg::Tovision();
            message.control_mode = 5;
            message.sight_mode = 1;
            message.hit_priority = {0, 0, 0, 0, 0, 0, 0, 0};
            message.expect_spin_degree = 0;
            message.dishit = {0, 0, 0, 0, 0, 0, 0, 0};
            this->publisher_->publish(message);
            RCLCPP_INFO(rclcpp::get_logger("ControlAutoAim"), "进入自瞄模式");

        } else {
            auto message = serial_interfaces::msg::Tovision();
            message.control_mode = 5;
            message.sight_mode = 1;
            message.hit_priority = {0, 0, 0, 0, 0, 0, 0, 0};
            message.expect_spin_degree = 0;
            message.dishit = {0, 0, 0, 0, 0, 0, 0, 0};
            this->publisher_->publish(message);
            RCLCPP_INFO(rclcpp::get_logger("ControlAutoAim"), "不进入自瞄模式");
        }

        return NodeStatus::SUCCESS;
    }

    rclcpp::Publisher<serial_interfaces::msg::Tovision>::SharedPtr publisher_;
    rclcpp::Node::SharedPtr node_;
};