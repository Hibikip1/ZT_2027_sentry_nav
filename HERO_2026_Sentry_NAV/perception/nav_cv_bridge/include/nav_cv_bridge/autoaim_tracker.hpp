#ifndef NAV_CV_BRIDGE__AUTOAIM_TRACKER_HPP_
#define NAV_CV_BRIDGE__AUTOAIM_TRACKER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_cv_bridge/color_info_map.hpp"
#include "interfaces/msg/global_target_array.hpp"
#include "interfaces/msg/tracker_output.hpp"
#include "interfaces/msg/mask_id.hpp"
namespace nav_cv_bridge {

struct TargetState {
    std::string id;

    // 自瞄数据
    rclcpp::Time last_autoaim_time{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Point autoaim_pos_map;

    // 雷达站数据
    rclcpp::Time last_radar_time{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Point radar_pos_map;

    // 滞回器状态
    bool using_radar{false};
};

class AutoaimTracker : public rclcpp::Node {
   public:
    explicit AutoaimTracker(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void autoaimCallback(interfaces::msg::TrackerOutput::SharedPtr msg);
    void radarCallback(interfaces::msg::GlobalTargetArray::SharedPtr msg);
    void timerCallback();

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<interfaces::msg::TrackerOutput>::SharedPtr
        autoaim_sub_;
    rclcpp::Subscription<interfaces::msg::GlobalTargetArray>::SharedPtr radar_sub_;

    rclcpp::Publisher<interfaces::msg::GlobalTargetArray>::SharedPtr
        global_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;
    rclcpp::Publisher<interfaces::msg::MaskID>::SharedPtr mask_id_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex targets_mutex_;
    std::unordered_map<std::string, TargetState> targets_;
    std::shared_ptr<colorMap::ColorMapModel> color_map_model_;
    // 参数
    double timeout_sec_{1.0};
    double radar_switch_dist_{5.5};
    double autoaim_switch_dist_{4.5};
};

}  // namespace nav_cv_bridge

#endif  // NAV_CV_BRIDGE__AUTOAIM_TRACKER_HPP_
