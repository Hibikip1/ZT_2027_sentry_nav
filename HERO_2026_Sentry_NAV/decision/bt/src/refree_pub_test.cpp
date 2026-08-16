#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "serial_interfaces/msg/refree.hpp"

using namespace std::chrono_literals;

class RefreeSimulatorNode : public rclcpp::Node {
   public:
    RefreeSimulatorNode() : Node("refree_simulator_node") {
        // 与 chassis_node 保持一致
        refree_pub_ = this->create_publisher<serial_interfaces::msg::Refree>(
            "refree_msg", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&RefreeSimulatorNode::timer_callback, this));

        // 声明需要模拟的参数
        this->declare_parameter("competition_type", 1);
        this->declare_parameter("competition_stage", 4);
        this->declare_parameter("stage_time", 300);  // 剩余300s
        this->declare_parameter("dart_target", 0);

        // 比赛环境和能量机关
        this->declare_parameter("small_energy_state", 0);
        this->declare_parameter("big_energy_state", 0);

        this->declare_parameter("rune_available", 1);

        this->declare_parameter("robot_id", 7);      // 7 蓝方哨兵
        this->declare_parameter("current_hp", 600);  // 哨兵当前血量
        this->declare_parameter("bullet_17mm", 100);
    }

   private:
    void timer_callback() {
        auto msg = serial_interfaces::msg::Refree();
        msg.competition_type = this->get_parameter("competition_type").as_int();
        msg.competition_stage =
            this->get_parameter("competition_stage").as_int();
        msg.stage_time = this->get_parameter("stage_time").as_int();
        msg.dart_target = this->get_parameter("dart_target").as_int();
        msg.small_energy_state =
            this->get_parameter("small_energy_state").as_int();
        msg.big_energy_state = this->get_parameter("big_energy_state").as_int();
        msg.energy_mechanism_available = this->get_parameter("rune_available").as_int();

        msg.robot_id = this->get_parameter("robot_id").as_int();
        msg.current_hp = this->get_parameter("current_hp").as_int();

        msg.bullet_17mm = this->get_parameter("bullet_17mm").as_int();

        refree_pub_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<serial_interfaces::msg::Refree>::SharedPtr refree_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefreeSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}