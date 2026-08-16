#include <behaviortree_cpp/blackboard.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <interfaces/msg/target_switch.hpp>
#include <interfaces/msg/target_switchs.hpp>
#include <memory>
#include <random>
#include <rclcpp/logging.hpp>
#include <serial_interfaces/msg/radar.hpp>
#include <serial_interfaces/msg/refree.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "behavior_node.hpp"
#include "cruiser_node.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::literals;

/**
 * @brief 黑板数据更新独立线程，专门负责从话题获取数据并更新黑板
 */
class BlackboardUpdater {
   public:
    BlackboardUpdater(Blackboard::Ptr blackboard, rclcpp::Node::SharedPtr node,
                      bool is_red_team)
        : blackboard_(blackboard),
          node_(node),
          running_(false),
          is_red_team_(is_red_team) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化订阅者
        // 裁判系统订阅
        refree_subscriber_ =
            node_->create_subscription<serial_interfaces::msg::Refree>(
                "refree_msg", 10,
                [this](const serial_interfaces::msg::Refree::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    // 比赛信息
                    blackboard_->set("competition_type", msg->competition_type);
                    blackboard_->set("competition_stage",
                                     msg->competition_stage);
                    blackboard_->set("stage_time", msg->stage_time);
                    blackboard_->set("enemy_outpost_status", msg->dart_target);
                    blackboard_->set("small_energy_state",
                                     msg->small_energy_state);
                    blackboard_->set("big_energy_state", msg->big_energy_state);

                    blackboard_->set("rune_available",msg->energy_mechanism_available);

                    // 己方血量同步（根据最新的自定义消息定义，已去除了敌我颜色判断）
                    blackboard_->set("friend_hero", msg->friend_hero);
                    blackboard_->set("friend_engineer", msg->friend_engineer);
                    blackboard_->set("friend_infantry3", msg->friend_infantry3);
                    blackboard_->set("friend_infantry4", msg->friend_infantry4);
                    blackboard_->set("friend_sentry", msg->friend_sentry);
                    blackboard_->set("friend_outpost", msg->friend_outpost);
                    blackboard_->set("friend_base", msg->friend_base);

                    // 敌方血量现在不处理
                    // ...

                    // 战场状态
                    blackboard_->set("status_supply_non_overlap",
                                     msg->status_supply_non_overlap);
                    blackboard_->set("status_supply_overlap",
                                     msg->status_supply_overlap);
                    blackboard_->set("status_supply_occupied",
                                     msg->status_supply_occupied);
                    blackboard_->set("energy_activation",
                                     msg->energy_activation);
                    blackboard_->set("highland_status", msg->highland_status);
                    blackboard_->set("trapezoid_status", msg->trapezoid_status);
                    blackboard_->set("dart_hit_time", msg->dart_hit_time);
                    blackboard_->set("dart_target", msg->dart_target);
                    blackboard_->set("center_control", msg->center_control);

                    // 本机状态
                    blackboard_->set("robot_id", msg->robot_id);
                    blackboard_->set("robot_level", msg->robot_level);
                    blackboard_->set("current_hp", msg->current_hp);
                    blackboard_->set("max_hp", msg->max_hp);
                    blackboard_->set("cooling_rate", msg->cooling_rate);
                    blackboard_->set("heat_limit", msg->heat_limit);
                    blackboard_->set("chassis_heat_limit",
                                     msg->chassis_heat_limit);

                    // 位置信息
                    blackboard_->set("position_x", msg->position_x);
                    blackboard_->set("position_y", msg->position_y);
                    blackboard_->set("orientation", msg->orientation);
                    blackboard_->set("bullet_17mm", msg->bullet_17mm);
                    blackboard_->set("bullet_42mm", msg->bullet_42mm);
                    blackboard_->set("remaining_gold", msg->remaining_gold);
                    blackboard_->set("projectile_allowance_fortress",
                                     msg->projectile_allowance_fortress);

                    // 队友位置
                    auto transform_teammate = [&](float raw_x, float raw_y,
                                                  float& out_x, float& out_y) {
                        geometry_msgs::msg::Point p_in;
                        p_in.x = raw_x;
                        p_in.y = raw_y;
                        p_in.z = 0.0;
                        geometry_msgs::msg::Point p_out = p_in;
                        try {
                            geometry_msgs::msg::TransformStamped t =
                                tf_buffer_->lookupTransform("map",
                                                            "global_red_map",
                                                            tf2::TimePointZero);
                            tf2::doTransform(p_in, p_out, t);
                        } catch (const tf2::TransformException& ex) {
                            // ignore or log
                        }
                        out_x = p_out.x;
                        out_y = p_out.y;
                    };

                    float t_hero_x, t_hero_y, t_eng_x, t_eng_y, t_inf3_x,
                        t_inf3_y, t_inf4_x, t_inf4_y;
                    transform_teammate(msg->hero_x, msg->hero_y, t_hero_x,
                                       t_hero_y);
                    transform_teammate(msg->engineer_x, msg->engineer_y,
                                       t_eng_x, t_eng_y);
                    transform_teammate(msg->infantry3_x, msg->infantry3_y,
                                       t_inf3_x, t_inf3_y);
                    transform_teammate(msg->infantry4_x, msg->infantry4_y,
                                       t_inf4_x, t_inf4_y);

                    blackboard_->set("hero_x", t_hero_x);
                    blackboard_->set("hero_y", t_hero_y);
                    blackboard_->set("engineer_x", t_eng_x);
                    blackboard_->set("engineer_y", t_eng_y);
                    blackboard_->set("infantry3_x", t_inf3_x);
                    blackboard_->set("infantry3_y", t_inf3_y);
                    blackboard_->set("infantry4_x", t_inf4_x);
                    blackboard_->set("infantry4_y", t_inf4_y);

                    // 哨兵状态
                    blackboard_->set("sentry_bullet_allowance",
                                     msg->sentry_bullet_allowance);
                    blackboard_->set("sentry_remote_bullet_count",
                                     msg->sentry_remote_bullet_count);
                    blackboard_->set("sentry_remote_hp_count",
                                     msg->sentry_remote_hp_count);
                    blackboard_->set("sentry_confirm_revive",
                                     msg->sentry_confirm_revive);
                    blackboard_->set("sentry_immediate_revive",
                                     msg->sentry_immediate_revive);
                    blackboard_->set("sentry_revive_cost",
                                     msg->sentry_revive_cost);

                    // 附加状态
                    blackboard_->set("additional_status_bit0",
                                     msg->additional_status_bit0);
                    blackboard_->set("additional_status_bits1_11",
                                     msg->additional_status_bits1_11);
                    blackboard_->set("additional_status_bits12_15",
                                     msg->additional_status_bits12_15);
                });
        // 雷达站消息订阅
        radar_subscriber_ =
            node_->create_subscription<serial_interfaces::msg::Radar>(
                "radar_msg", 10,
                [this](const serial_interfaces::msg::Radar::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);

                    geometry_msgs::msg::Point p_map = msg->robot_1_position;
                    try {
                        geometry_msgs::msg::TransformStamped t =
                            tf_buffer_->lookupTransform("map", "global_red_map",
                                                        tf2::TimePointZero);
                        tf2::doTransform(msg->robot_1_position, p_map, t);
                    } catch (const tf2::TransformException& ex) {
                        RCLCPP_WARN_THROTTLE(
                            node_->get_logger(), *node_->get_clock(), 1000,
                            "Could not transform radar_msg: %s", ex.what());
                    }

                    // 更新敌方坐标信息到黑板
                    blackboard_->set("robot_1_x", p_map.x);
                    blackboard_->set("robot_1_y", p_map.y);

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "黑板雷达数据更新: 敌方位置=(%f, %f)", p_map.x,
                                 p_map.y);
                });
        // 触发式消息订阅
        trigger_subscriber_ =
            node_->create_subscription<geometry_msgs::msg::PointStamped>(
                "trigger_msg", 10,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);

                    // 记录上一帧的原始数据，用于比较是否有新触发
                    static double last_raw_trigger_x = 0.0;
                    static double last_raw_trigger_y = 0.0;

                    // 仅当原始坐标均不为0（视为有效坐标），且与上一帧原始坐标不一致时才被判定为新的触发式消息
                    if ((msg->point.x != 0.0 || msg->point.y != 0.0) &&
                        (msg->point.x != last_raw_trigger_x ||
                         msg->point.y != last_raw_trigger_y)) {
                        // Transform to map frame
                        geometry_msgs::msg::Point p_map = msg->point;
                        try {
                            geometry_msgs::msg::TransformStamped t =
                                tf_buffer_->lookupTransform(
                                    "map",
                                    msg->header.frame_id.empty()
                                        ? "global_red_map"
                                        : msg->header.frame_id,
                                    tf2::TimePointZero);
                            tf2::doTransform(msg->point, p_map, t);
                        } catch (const tf2::TransformException& ex) {
                            RCLCPP_WARN_THROTTLE(
                                node_->get_logger(), *node_->get_clock(), 1000,
                                "Could not transform trigger_msg: %s",
                                ex.what());
                        }

                        // 更新触发点到黑板
                        blackboard_->set("trigger_point_x", p_map.x);
                        blackboard_->set("trigger_point_y", p_map.y);

                        // 黑板上的标志位 trigger_flag 置 1 证明有触发式消息
                        blackboard_->set("trigger_flag", 1);

                        // 更新上一帧原始数据
                        last_raw_trigger_x = msg->point.x;
                        last_raw_trigger_y = msg->point.y;

                        RCLCPP_INFO(node_->get_logger(),
                                    "产生新触发式消息，更新黑板触发点: (%.3f, "
                                    "%.3f), flag=1",
                                    p_map.x, p_map.y);
                    }
                });

        RCLCPP_INFO(node_->get_logger(), "黑板数据更新器已创建");
    }

    // 启动更新线程
    void start() {
        running_ = true;
        update_thread_ = std::thread(&BlackboardUpdater::update_loop, this);
        RCLCPP_INFO(node_->get_logger(), "黑板数据更新线程已启动");
    }

    // 停止更新线程
    void stop() {
        running_ = false;
        if (update_thread_.joinable()) {
            update_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "黑板数据更新线程已停止");
    }

   private:
    // 更新循环，在独立线程中运行
    void update_loop() {
        rclcpp::Rate rate(20);  // 20Hz的更新频率
        while (running_ && rclcpp::ok()) {
            // 处理ROS消息队列
            rclcpp::spin_some(node_);
            rate.sleep();
        }
    }

    Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> running_;
    std::thread update_thread_;
    std::mutex mutex_;

    // 订阅者
    rclcpp::Subscription<serial_interfaces::msg::Refree>::SharedPtr
        refree_subscriber_;
    rclcpp::Subscription<serial_interfaces::msg::Radar>::SharedPtr
        radar_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        trigger_subscriber_;

    // 判断本局比赛红方还是蓝方
    // true：红方 false：蓝方
    bool is_red_team_ = true;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
/**
 * @brief 血量变化率监测独立线程
 */
class hp_spin_watcher {
   public:
    hp_spin_watcher(Blackboard::Ptr blackboard, rclcpp::Node::SharedPtr node)
        : blackboard_(blackboard), node_(node), running_(false) {
        // 创建消息发布者
        hp_danger_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
            "hp_danger_status", 10);

        RCLCPP_INFO(node_->get_logger(), "血量变化率监测器已创建");
    }

    // 启动监测线程
    void start() {
        running_ = true;
        watch_thread_ = std::thread(&hp_spin_watcher::watch_loop, this);
        RCLCPP_INFO(node_->get_logger(), "血量变化率监测线程已启动");
    }

    // 停止监测线程
    void stop() {
        running_ = false;
        if (watch_thread_.joinable()) {
            watch_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "血量变化率监测线程已停止");
    }

   private:
    struct HpRecord {
        std::chrono::system_clock::time_point timestamp;
        int hp;
    };

    struct TimeComparator {
        bool operator()(const HpRecord& a, const HpRecord& b) const {
            return a.timestamp > b.timestamp;  // 最小堆（堆顶是最旧记录）
        }
    };

    // 监测循环，在独立线程中运行
    void watch_loop() {
        rclcpp::Rate rate(10);  // 10Hz的监测频率

        // 保存最新记录的变量
        HpRecord latest_record{std::chrono::system_clock::now(), 0};

        while (running_ && rclcpp::ok()) {
            // 获取当前血量
            int current_hp = 0;
            if (blackboard_->get("current_hp", current_hp)) {
                auto now = std::chrono::system_clock::now();

                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);

                    // 更新最新记录
                    latest_record = {now, current_hp};

                    // 清理过期数据（超过2秒的记录）
                    while (!hp_queue_.empty()) {
                        auto elapsed = std::chrono::duration_cast<
                                           std::chrono::milliseconds>(
                                           now - hp_queue_.top().timestamp)
                                           .count();
                        if (elapsed > 2000) {
                            hp_queue_.pop();
                        } else {
                            break;  // 如果堆顶元素没过期，后面的也不会过期
                        }
                    }

                    // 添加新记录到队列
                    hp_queue_.push(latest_record);

                    // 计算血量变化
                    if (!hp_queue_.empty()) {
                        // 获取最旧记录（堆顶）
                        const auto& oldest = hp_queue_.top();

                        // 使用先前保存的最新记录
                        const auto& newest = latest_record;

                        // 计算时间差
                        auto time_diff =
                            std::chrono::duration_cast<
                                std::chrono::milliseconds>(newest.timestamp -
                                                           oldest.timestamp)
                                .count();

                        // 只有当最新和最旧记录不同时才计算血量变化
                        if (time_diff > 0) {
                            // 计算血量变化（旧血量 - 新血量），正值表示血量减少
                            int hp_change = oldest.hp - newest.hp;

                            // RCLCPP_INFO(node_->get_logger(),
                            // "最旧记录(%ld毫秒前): %d, 最新记录: %d, 差值:
                            // %d", time_diff, oldest.hp, newest.hp, hp_change);

                            // 如果两秒内血量减少超过30
                            if (hp_change > 30 && time_diff <= 2000) {
                                RCLCPP_WARN(
                                    node_->get_logger(),
                                    "检测到危险血量变化！%ld毫秒内减少%d点血量",
                                    time_diff, hp_change);

                                // 发布危险状态消息
                                auto msg =
                                    std::make_unique<std_msgs::msg::Bool>();
                                msg->data = true;
                                hp_danger_publisher_->publish(*msg);
                            }
                        }
                    }
                }
            }

            rate.sleep();
        }
    }

    Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> running_;
    std::thread watch_thread_;

    // 使用优先队列实现最小堆，按时间戳排序（最旧的记录在堆顶）
    std::priority_queue<HpRecord, std::vector<HpRecord>, TimeComparator>
        hp_queue_;
    std::mutex queue_mutex_;  // 保护血量记录的互斥锁

    // 危险状态发布者
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hp_danger_publisher_;
};

/**
 * @brief 云台控制：上位机决策端
 */
class GimbalSerialNode {
   public:
    explicit GimbalSerialNode(rclcpp::Node::SharedPtr node,
                              BT::Blackboard::Ptr blackboard)
        : node_(node), blackboard_(blackboard), running_(false) {
        // 创建发布者
        gimbal_pub_ = node_->create_publisher<interfaces::msg::TargetSwitchs>(
            "gimbal_decision", 10);

        RCLCPP_INFO(node_->get_logger(), "云台控制节点已创建");
    }

    // 启动更新线程
    void start() {
        running_ = true;
        update_thread_ = std::thread(&GimbalSerialNode::update_loop, this);
        RCLCPP_INFO(node_->get_logger(), "云台控制线程已启动");
    }

    // 停止更新线程
    void stop() {
        running_ = false;
        if (update_thread_.joinable()) {
            update_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "云台控制线程已停止");
    }

   private:
    void update_loop() {
        rclcpp::Rate rate(10);  // 10Hz的更新频率

        while (running_ && rclcpp::ok()) {
            auto msg = interfaces::msg::TargetSwitchs();
            msg.header.stamp = node_->now();
            // msg.ts.resize(7);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                // 从黑板读取数据
                std::vector<interfaces::msg::TargetSwitch> targets;
                if (blackboard_->get("targets", targets)) {
                    msg.ts = targets;
                }
            }

            // 发布消息
            gimbal_pub_->publish(msg);

            rate.sleep();
        }
    }

    rclcpp::Node::SharedPtr node_;
    BT::Blackboard::Ptr blackboard_;
    std::atomic<bool> running_;
    std::thread update_thread_;
    std::mutex mutex_;
    rclcpp::Publisher<interfaces::msg::TargetSwitchs>::SharedPtr gimbal_pub_;
};

/**
 * @brief 哨兵攻击状态监控独立线程
 */
class sentry_attack_watcher {
   public:
    enum class State {
        STATE_1,  // 监控敌方哨兵血量
        STATE_2,  // 等待敌方哨兵血量恢复
        STATE_3   // 倒计时状态
    };

    sentry_attack_watcher(Blackboard::Ptr blackboard,
                          rclcpp::Node::SharedPtr node)
        : blackboard_(blackboard),
          node_(node),
          running_(false),
          current_state_(State::STATE_1),
          counter_(0) {
        // 初始化can_attack_sentry为true
        blackboard_->set("can_attack_sentry", true);

        RCLCPP_INFO(node_->get_logger(), "哨兵攻击状态监控器已创建");
    }

    // 启动监测线程
    void start() {
        running_ = true;
        watch_thread_ = std::thread(&sentry_attack_watcher::watch_loop, this);
        RCLCPP_INFO(node_->get_logger(), "哨兵攻击状态监控线程已启动");
    }

    // 停止监测线程
    void stop() {
        running_ = false;
        if (watch_thread_.joinable()) {
            watch_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "哨兵攻击状态监控线程已停止");
    }

   private:
    // 监测循环，在独立线程中运行
    void watch_loop() {
        rclcpp::Rate rate(2);  // 0.5秒一次回调 (2Hz)

        while (running_ && rclcpp::ok()) {
            switch (current_state_) {
                case State::STATE_1:
                    handle_state_1();
                    break;
                case State::STATE_2:
                    handle_state_2();
                    break;
                case State::STATE_3:
                    handle_state_3();
                    break;
            }

            rate.sleep();
        }
    }

    // 状态1：监控敌方哨兵血量
    void handle_state_1() {
        int enemy_sentry_hp = 0;
        if (blackboard_->get("enemy_sentry", enemy_sentry_hp)) {
            if (enemy_sentry_hp == 0) {
                // 敌方哨兵血量为0，禁止攻击并进入状态2
                blackboard_->set("can_attack_sentry", false);
                current_state_ = State::STATE_2;
                RCLCPP_INFO(node_->get_logger(),
                            "状态1->状态2: 敌方哨兵血量为0，禁止攻击");
            }
            // 血量不为0时不做处理，保持状态1
        }
    }

    // 状态2：等待敌方哨兵血量恢复
    void handle_state_2() {
        int enemy_sentry_hp = 0;
        if (blackboard_->get("enemy_sentry", enemy_sentry_hp)) {
            if (enemy_sentry_hp == 0) {
                // 血量仍为0，保持状态2
                return;
            } else if (enemy_sentry_hp == 40) {
                // 血量恢复到40，进入状态3并重置计时器
                current_state_ = State::STATE_3;
                counter_ = 0;  // 重新计时
                RCLCPP_INFO(
                    node_->get_logger(),
                    "状态2->状态3: 敌方哨兵复活，血量恢复到40，开始倒计时");
            } else {
                // 血量不为0也不为20，回到状态1
                blackboard_->set("can_attack_sentry", true);
                current_state_ = State::STATE_1;
                RCLCPP_INFO(node_->get_logger(),
                            "状态2->状态1: 敌方哨兵血量为%d", enemy_sentry_hp);
            }
        }
    }

    // 状态3：倒计时状态
    void handle_state_3() {
        int enemy_sentry_hp = 0;
        counter_++;  // 每次进入状态3就递增计数器

        if (blackboard_->get("enemy_sentry", enemy_sentry_hp)) {
            // 检查是否需要退出状态3
            if (enemy_sentry_hp != 40 ||
                counter_ >= 120) {  // 血量不为40或者超过60秒
                // 允许攻击并返回状态1
                blackboard_->set("can_attack_sentry", true);
                current_state_ = State::STATE_1;

                if (counter_ >= 120) {
                    RCLCPP_INFO(node_->get_logger(),
                                "状态3->状态1: 倒计时60秒结束，允许攻击");
                } else {
                    RCLCPP_INFO(node_->get_logger(),
                                "状态3->状态1: 敌方哨兵血量变为%d，允许攻击",
                                enemy_sentry_hp);
                }
                counter_ = 0;  // 重置计数器
            } else {
                // 继续在状态3中等待，计数器继续累加
                RCLCPP_DEBUG(node_->get_logger(), "状态3: 倒计时中... (%d/120)",
                             counter_);
            }
        }
    }

    Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> running_;
    std::thread watch_thread_;

    State current_state_;  // 当前状态
    int counter_;          // 状态3的计时器
};

#include <array>
#include <vector>
#include <utility>
#include <color_info_map.hpp>
/**
 * @brief 位置判定独立线程
 */
class AdjustPositions {
   public:
     AdjustPositions(Blackboard::Ptr blackboard, rclcpp::Node::SharedPtr node)
        : blackboard_(blackboard), node_(node), running_(false) {
        // 1. 改为一维数组 std::vector<int64_t>
        node ->declare_parameter("color_map_yaml_path", "");
        node ->declare_parameter("color_map_path", "");
        std::string color_map_yaml_path;
        std::string color_map_path;
        node->get_parameter("color_map_yaml_path", color_map_yaml_path);
        node->get_parameter("color_map_path", color_map_path);
        
        color_map_model_ = std::make_shared<colorMap::ColorMapModel>(color_map_path, color_map_yaml_path);
           
        RCLCPP_INFO(node_->get_logger(), "位置判定器已创建");
    }

    // 启动判定线程
    void start() {
        running_ = true;
        update_thread_ = std::thread(&AdjustPositions::update_loop, this);
        RCLCPP_INFO(node_->get_logger(), "位置判定线程已启动");
    }

    // 停止判定线程
    void stop() {
        running_ = false;
        if (update_thread_.joinable()) {
            update_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "位置判定线程已停止");
    }

   private:
    // 判定函数，具体逻辑由用户实现
    uint8_t evaluatePosition(double x, double y) {
        unsigned mx,my;
       if( !color_map_model_->World2ColorMap(x,y,mx,my)) return 0;
       return  color_map_model_->GetColorMapInfo(mx,my);
        
    }

    // 判定循环，在独立线程中运行
    void update_loop() {
        rclcpp::Rate rate(10);  // 10Hz更新频率 (可根据需要调整)

        while (running_ && rclcpp::ok()) {
            // 初始化长度为6的状态数组
            int res_infantry3=0;
            int res_infantry4=0;
      

            // 3. 判断 Infantry3
            double infantry3_x = 0, infantry3_y = 0;
            if(blackboard_->get("infantry3_x", infantry3_x) && blackboard_->get("infantry3_y", infantry3_y)){
                res_infantry3 = evaluatePosition(infantry3_x, infantry3_y);
                if(res_infantry3 == 1) {
                    blackboard_->set("infantry3_event_triggered", true);
                } else {
                    blackboard_->set("infantry3_event_triggered", false);
                }
            }

            // 4. 判断 Infantry4
            double infantry4_x = 0, infantry4_y = 0;
            if(blackboard_->get("infantry4_x", infantry4_x) && blackboard_->get("infantry4_y", infantry4_y)){
                res_infantry4 = evaluatePosition(infantry4_x, infantry4_y);
                if(res_infantry4 == 1) {
                    blackboard_->set("infantry4_event_triggered", true);
                } else {
                    blackboard_->set("infantry4_event_triggered", false);
                }
            }

            // 将拼装好的最终数组写回黑板

            rate.sleep();
        }
    }
    Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> running_;
    std::thread update_thread_;
    std::shared_ptr<colorMap::ColorMapModel> color_map_model_;
};
