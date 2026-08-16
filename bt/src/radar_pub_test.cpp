#include "geometry_msgs/msg/point.hpp"
#include "interfaces/msg/global_target_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RadarSimulatorNode : public rclcpp::Node {
   public:
    RadarSimulatorNode() : Node("radar_simulator_node") {
        // 创建雷达消息发布器，与 chassis_node 保持一致
        radar_pub_ = this->create_publisher<interfaces::msg::GlobalTargetArray>(
            "/tracker/radar", rclcpp::SensorDataQoS());

        // 创建定时器（100ms发布周期 = 10Hz）
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RadarSimulatorNode::timer_callback, this));

        // 声明所有的坐标参数 (对手 hero, engineer, infantry_3, infantry_4,
        // sentry)
        this->declare_parameter("hero_x", 5.0);
        this->declare_parameter("hero_y", 0.0);
        this->declare_parameter("engineer_x", 0.0);
        this->declare_parameter("engineer_y", 0.0);
        this->declare_parameter("infantry_3_x", 0.0);
        this->declare_parameter("infantry_3_y", 0.0);
        this->declare_parameter("infantry_4_x", 0.0);
        this->declare_parameter("infantry_4_y", 0.0);
        this->declare_parameter("sentry_x", 0.0);
        this->declare_parameter("sentry_y", 0.0);
        // 是否将不在原点(0,0)的位置作为有交战目标加入
        this->declare_parameter("enable_hero", true);
        this->declare_parameter("enable_sentry", false);
    }

   private:
    void timer_callback() {
        interfaces::msg::GlobalTargetArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        auto add_target = [&](double x, double y, const std::string& id_name,
                              bool enabled) {
            if (enabled) {
                interfaces::msg::GlobalTarget target;
                target.id = id_name;
                target.position.x = x;
                target.position.y = y;
                target.position.z = 0.0;
                msg.targets.push_back(target);
            }
        };

        add_target(this->get_parameter("hero_x").as_double(),
                   this->get_parameter("hero_y").as_double(), "1",
                   this->get_parameter("enable_hero").as_bool());

        add_target(this->get_parameter("sentry_x").as_double(),
                   this->get_parameter("sentry_y").as_double(), "SENTRY",
                   this->get_parameter("enable_sentry").as_bool());

        if (!msg.targets.empty()) {
            radar_pub_->publish(msg);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::GlobalTargetArray>::SharedPtr radar_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}