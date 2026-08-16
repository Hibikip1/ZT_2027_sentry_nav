#include "nav_cv_bridge/autoaim_tracker.hpp"

#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav_cv_bridge {

AutoaimTracker::AutoaimTracker(const rclcpp::NodeOptions& options)
    : Node("autoaim_tracker", options) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and initialize parameters
    this->declare_parameter("timeout_sec", 1.0);
    this->declare_parameter("radar_switch_dist", 5.5);
    this->declare_parameter("autoaim_switch_dist", 4.5);
    this->declare_parameter(
        "color_map_file",
        "/home/dji/hero2026_-sentry/src/planning/map_server/map/map.png");
    this->declare_parameter(
        "yaml_file",
        "/home/dji/hero2026_-sentry/src/planning/map_server/map/map.yaml");
    std::string color_map_file, yaml_file;
    this->get_parameter("yaml_file", yaml_file);
    this->get_parameter("color_map_file", color_map_file);
    color_map_model_ =
        std::make_shared<colorMap::ColorMapModel>(color_map_file, yaml_file);
    autoaim_switch_dist_ =
        this->get_parameter("autoaim_switch_dist").as_double();
    radar_switch_dist_ = this->get_parameter("radar_switch_dist").as_double();
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();
    mask_id_pub_ = this->create_publisher<interfaces::msg::MaskID>(
        "MaskID", rclcpp::QoS(10));
    autoaim_sub_ = this->create_subscription<interfaces::msg::TrackerOutput>(
        "/tracker/autoaim", rclcpp::SensorDataQoS(),
        std::bind(&AutoaimTracker::autoaimCallback, this,
                  std::placeholders::_1));

    radar_sub_ = this->create_subscription<interfaces::msg::GlobalTargetArray>(
        "/tracker/radar", rclcpp::SensorDataQoS(),
        std::bind(&AutoaimTracker::radarCallback, this, std::placeholders::_1));

    global_target_pub_ =
        this->create_publisher<interfaces::msg::GlobalTargetArray>(
            "/global_targets", rclcpp::SensorDataQoS());

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/global_targets_marker", rclcpp::QoS(1));

    // 5Hz timer for publishing the fused target info
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&AutoaimTracker::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "AutoaimTracker (Fusion) node initialized. Publishing at 5Hz");
}

void AutoaimTracker::autoaimCallback(
    interfaces::msg::TrackerOutput::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped baselink_to_map;
    try {
        // Use the timestamp from the message, timeout logic applied
        baselink_to_map =
            tf_buffer_->lookupTransform("map", "base_link", msg->header.stamp,
                                        rclcpp::Duration::from_seconds(0.05));
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Autoaim TF Error: %s", ex.what());
        return;
    }

    std::lock_guard<std::mutex> lock(targets_mutex_);
    rclcpp::Time now_stamp = msg->header.stamp;

    for (const auto& target : msg->targets) {
        std::string t_id = target.id;

        geometry_msgs::msg::Point p_base = target.position;
        geometry_msgs::msg::Point p_map;
        tf2::doTransform(p_base, p_map, baselink_to_map);

        targets_[t_id].id = t_id;
        targets_[t_id].last_autoaim_time = now_stamp;
        targets_[t_id].autoaim_pos_map = p_map;
    }
}

void AutoaimTracker::radarCallback(
    interfaces::msg::GlobalTargetArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(targets_mutex_);
    rclcpp::Time now_stamp = msg->header.stamp;

    // Simulate radar map frame data
    geometry_msgs::msg::TransformStamped global_to_map;
    bool can_transform = true;
    try {
        global_to_map = tf_buffer_->lookupTransform(
            "map", msg->header.frame_id, msg->header.stamp,
            rclcpp::Duration::from_seconds(0.05));
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Radar TF Error: %s", ex.what());
        can_transform = false;
    }

    for (const auto& target : msg->targets) {
        std::string t_id = target.id;

        geometry_msgs::msg::Point p_global = target.position;
        geometry_msgs::msg::Point p_map;
        if (can_transform) {
            tf2::doTransform(p_global, p_map, global_to_map);
        } else {
            p_map = p_global;  // Fallback if TF is unavailable
        }

        targets_[t_id].id = t_id;
        targets_[t_id].last_radar_time = now_stamp;
        targets_[t_id].radar_pos_map = p_map;
    }
}

void AutoaimTracker::timerCallback() {
    std::lock_guard<std::mutex> lock(targets_mutex_);
    rclcpp::Time current_time = this->now();

    // Check my position at current time via TF for hysteresis decision
    bool has_self_pos = false;
    geometry_msgs::msg::Point self_pos_map;
    try {
        auto transform =
            tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        self_pos_map.x = transform.transform.translation.x;
        self_pos_map.y = transform.transform.translation.y;
        self_pos_map.z = transform.transform.translation.z;
        has_self_pos = true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Timer TF Error (Self Pos): %s", ex.what());
    }

    auto out_msg = std::make_shared<interfaces::msg::GlobalTargetArray>();
    out_msg->header.frame_id = "map";
    out_msg->header.stamp = current_time;

    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (auto it = targets_.begin(); it != targets_.end();) {
        TargetState& state = it->second;

        // Note: checking nanoseconds > 0 is necessary if uninitialized Time
        // object is used.
        bool autoaim_valid =
            (state.last_autoaim_time.nanoseconds() > 0) &&
            ((current_time - state.last_autoaim_time).seconds() < timeout_sec_);
        bool radar_valid =
            (state.last_radar_time.nanoseconds() > 0) &&
            ((current_time - state.last_radar_time).seconds() < timeout_sec_);

        // Double timeout -> erase
        if (!autoaim_valid && !radar_valid) {
            it = targets_.erase(it);
            continue;
        }

        bool use_radar_this_tick = state.using_radar;
        geometry_msgs::msg::Point final_pos;

        if (has_self_pos) {
            double dx = 0.0, dy = 0.0;
            if (autoaim_valid) {
                dx = state.autoaim_pos_map.x - self_pos_map.x;
                dy = state.autoaim_pos_map.y - self_pos_map.y;
            } else {
                dx = state.radar_pos_map.x - self_pos_map.x;
                dy = state.radar_pos_map.y - self_pos_map.y;
            }
            double dist = std::hypot(dx, dy);

            if (state.using_radar) {
                // 如果上一次用的是雷达，那么需要满足距离条件才切回自瞄，否则继续用雷达
                if (autoaim_valid &&
                    (dist < autoaim_switch_dist_ || !radar_valid)) {
                    use_radar_this_tick = false;  // switch to autoaim
                }
            } else {
                // 如果上一次用的是自瞄，那么需要满足距离条件才切回雷达，否则继续用自瞄
                if (radar_valid &&
                    (dist > radar_switch_dist_ || !autoaim_valid)) {
                    use_radar_this_tick = true;  // switch to radar
                }
            }
        } else {
            // fallback: autoaim first
            use_radar_this_tick = !autoaim_valid;
        }

        // Fallback checks incase logic failed
        if (use_radar_this_tick && !radar_valid && autoaim_valid) {
            // 想用雷达，但雷达刚超时，自瞄还活着，强切自瞄
            use_radar_this_tick = false;
        } else if (!use_radar_this_tick && !autoaim_valid && radar_valid) {
            // 想用自瞄，但这帧自瞄丢了，雷达还在，强切雷达
            use_radar_this_tick = true;
        }

        // state commit
        state.using_radar = use_radar_this_tick;

        interfaces::msg::GlobalTarget g_target;
        g_target.id = state.id;

        if (use_radar_this_tick) {
            final_pos = state.radar_pos_map;
            g_target.position = final_pos;
        } else {
            final_pos = state.autoaim_pos_map;
            g_target.position = final_pos;
        }
        if (g_target.id == "2" && color_map_model_) {
            std::cout << "recieve_eginerr" << std::endl;
            unsigned int mx, my;
            if (color_map_model_->World2ColorMap(g_target.position.x,
                                                 g_target.position.y, mx, my)) {
                if (color_map_model_->GetColorMapInfo(mx, my) ==
                    colorMap::RED) {
                    interfaces::msg::MaskID mask_msg;
                    mask_msg.mask_id.push_back(g_target.id);
                    mask_id_pub_->publish(mask_msg);
                    RCLCPP_INFO(this->get_logger(),
                                "目标 %s 位于豁免区域，发布 MaskID=1",
                                g_target.id.c_str());
                    ++it;
                    continue;
                }
            }
        }

        out_msg->targets.push_back(g_target);

        // visualization
        visualization_msgs::msg::Marker marker;
        marker.header = out_msg->header;
        marker.ns = "global_targets";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = final_pos;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        if (use_radar_this_tick) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // blue for radar
        } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;  // red for autoaim
        }
        marker.color.a = 0.8;
        marker.lifetime =
            rclcpp::Duration::from_seconds(0.3);  // survive slightly over timer

        marker_array.markers.push_back(marker);

        // 文字可视化：用于展示目标的 ID
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = out_msg->header;
        text_marker.ns = "global_targets_id";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position = final_pos;
        text_marker.pose.position.z = 0.9;  // 悬浮在球体 (0.5) 的上方
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.25;  // 字体大小
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;    // 纯白色
        text_marker.text = state.id;  // 打印名称（比如 "hero"）
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

        marker_array.markers.push_back(text_marker);

        ++it;
    }

    if (!out_msg->targets.empty()) {
        global_target_pub_->publish(*out_msg);
        marker_pub_->publish(marker_array);
    }
}

}  // namespace nav_cv_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_cv_bridge::AutoaimTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}