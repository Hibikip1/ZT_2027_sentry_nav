#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <std_msgs/msg/bool.hpp>
#include <thread>

#include "attack_rune_nodes.hpp"
#include "behavior_node.hpp"
#include "cruiser_node.hpp"
#include "guard_with_attack_node.hpp"
#include "thread_node.hpp"
#include "zone_node.hpp"
using namespace std::literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_referee_subscriber");

    // 声明参数
    node->declare_parameter<std::string>(
        "xml_config_path", "src/decision/bt/config/xml/RMUC_test3.xml");
    node->declare_parameter<bool>("is_red_team_", true);
    node->declare_parameter<int>("cruise_state", 0);
    node->declare_parameter<int>("global_total_points", 3);

    // 获取参数值
    std::string xml_path = node->get_parameter("xml_config_path").as_string();
    bool is_red_team = node->get_parameter("is_red_team_").as_bool();
    int cruise_state = node->get_parameter("cruise_state").as_int();
    int global_total_points =
        node->get_parameter("global_total_points").as_int();

    Blackboard::Ptr blackboard = Blackboard::create();

    // 初始化黑板数据 - 设置默认值防止初始时空值
    blackboard->set("node", node);
    blackboard->set<int>("cruise_state", cruise_state);
    blackboard->set<int>("global_total_points", global_total_points);

    blackboard->set<int>("competition_type", 0);
    blackboard->set<int>("competition_stage", 0);
    blackboard->set<int>("stage_time", 0);
    blackboard->set<int>("enemy_outpost_status", 0);
    blackboard->set<int>("small_energy_state", 0);
    blackboard->set<int>("big_energy_state", 0);

    blackboard->set<int>("rune_available", 1);

    blackboard->set<int>("friend_hero", 400);
    blackboard->set<int>("friend_engineer", 400);
    blackboard->set<int>("friend_infantry3", 400);
    blackboard->set<int>("friend_infantry4", 400);
    blackboard->set<int>("friend_sentry", 400);
    blackboard->set<int>("friend_outpost", 1500);
    blackboard->set<int>("friend_base", 5000);

    blackboard->set<int>("enemy_hero", 400);
    blackboard->set<int>("enemy_engineer", 400);
    blackboard->set<int>("enemy_infantry3", 400);
    blackboard->set<int>("enemy_infantry4", 400);
    blackboard->set<int>("enemy_sentry", 400);
    blackboard->set<int>("enemy_outpost", 1500);
    blackboard->set<int>("enemy_base", 5000);

    blackboard->set<int>("robot_id", 0);
    blackboard->set<int>("bullet_17mm", 0);
    blackboard->set<int>("projectile_allowance_fortress", 0);
    blackboard->set<int>("current_hp", 0);
    blackboard->set<double>("robot_1_x", 0.0);
    blackboard->set<double>("robot_1_y", 0.0);

    blackboard->set<double>("trigger_point_x", 0.0);
    blackboard->set<double>("trigger_point_y", 0.0);
    blackboard->set<int>("trigger_flag", 0);
    blackboard->set<double>("x_offset", 0.0);
    blackboard->set<double>("y_offset", 0.0);

    blackboard->set<float>("hero_x", 0.0);
    blackboard->set<float>("hero_y", 0.0);
    blackboard->set<float>("engineer_x", 0.0);
    blackboard->set<float>("engineer_y", 0.0);
    blackboard->set<float>("infantry3_x", 0.0);
    blackboard->set<float>("infantry3_y", 0.0);
    blackboard->set<float>("infantry4_x", 0.0);
    blackboard->set<float>("infantry4_y", 0.0);

    blackboard->set("can_attack_sentry", true);

    blackboard->set<int>("small_rune_flag_1", 0);
    blackboard->set<int>("small_rune_flag_2", 0);
    blackboard->set<int>("big_rune_flag_1", 0);
    blackboard->set<int>("big_rune_flag_2", 0);
    blackboard->set<int>("big_rune_flag_3", 0);

    blackboard->set<int>("is_recovering", 0);

    // 独立线程创建并启动黑板更新器
    auto blackboard_updater =
        std::make_shared<BlackboardUpdater>(blackboard, node, is_red_team);
    blackboard_updater->start();

    // 创建并启动血量变化率监测器
    auto hp_watcher = std::make_shared<hp_spin_watcher>(blackboard, node);
    hp_watcher->start();

    // 创建并启动独立自瞄状态广播节点
    // auto gimbal_serial_node =
    //     std::make_shared<GimbalSerialNode>(node, blackboard);
    // gimbal_serial_node->start();  // 启动线程

    // auto position_adjuster = std::make_shared<AdjustPositions>(blackboard,
    // node); position_adjuster->start();  // 启动位置判定线程
    // 启动敌方哨兵血量监控进程
    // auto sentry_watcher = std::make_shared<sentry_attack_watcher>(blackboard,
    // node); sentry_watcher->start();

    BehaviorTreeFactory factory;
    factory.registerNodeType<CheckValueNode>("CheckValue");
    factory.registerNodeType<DoubleCheckValueNode>("DoubleCheckValue");
    factory.registerNodeType<LogMessageNode>("LogMessage");
    factory.registerNodeType<CheckCountdownNode>("CheckCountdown");
    factory.registerNodeType<ControlAutoAim>("ControlAutoAim");
    factory.registerNodeType<NavNode>("Nav");
    factory.registerNodeType<CruiserStateManagerNode>("CruiserStateManager");
    factory.registerNodeType<GuardianNode>("Guardian");
    factory.registerNodeType<AttackNavNode>("AttackNav");
    factory.registerNodeType<AutoAimManagerNode>("AutoAimManager");
    factory.registerNodeType<ClearFlagNode>("ClearFlagNode");
    factory.registerNodeType<GuardWithAttackNode>("GuardWithAttack");
    factory.registerNodeType<PublishNavOutputNode>("PublishNavOutput");
    factory.registerNodeType<RuneStateMachineNode>("RuneStateMachine");
    factory.registerNodeType<bt_nodes::CheckZoneNode>("CheckZone");
    factory.registerNodeType<bt_nodes::SetSpinNode>("SetSpin");

    RCLCPP_INFO(rclcpp::get_logger("refree_sub"), "行为树节点登记成功");
    auto tree = factory.createTreeFromFile(xml_path, blackboard);

    // 主线程专注行为树执行
    rclcpp::Rate tree_rate(10);
    while (rclcpp::ok()) {
        // 持续推进行为树
        const auto status = tree.tickWhileRunning();
        tree_rate.sleep();
    }

    // 清理资源
    blackboard_updater->stop();
    hp_watcher->stop();  // 停止血量监测器
    // gimbal_serial_node->stop();  // 停止线程
    rclcpp::shutdown();
    return 0;
}