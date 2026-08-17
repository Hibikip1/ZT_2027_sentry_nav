// zone_manager: 地图区域管理器
// 读取 zones.yaml 配置, 订阅机器人位姿判断所在区域,
// 发布:
//   1. visualization_msgs/MarkerArray -> /zones_markers (rviz 可视化)
//   2. interfaces/msg/ZoneInfo         -> /zone_info (给决策层行为树)
//
// 区域任务属性: spin(小陀螺), face_hole(正对洞口), nav_yaw(目标朝向),
//               speed_limit(限速)

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/zone_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace zone_mgr {

struct ZoneTask {
    bool spin_enabled = true;
    bool face_hole = false;
    double nav_yaw = 0.0;
    double speed_limit = 0.0;  // 0 = 不限速
    double stop_distance = 1.5;
};

struct Zone {
    std::string name;
    std::string shape;  // "rectangle" | "polygon"
    std::vector<geometry_msgs::msg::Point> points;  // 顶点(世界坐标)
    double center_x = 0.0, center_y = 0.0;
    double yaw = 0.0;
    ZoneTask tasks;
};

class ZoneManagerNode : public rclcpp::Node {
   public:
    ZoneManagerNode() : Node("zone_manager") {
        this->declare_parameter<std::string>("zones_config_path", "");
        std::string cfg_path = this->get_parameter("zones_config_path").as_string();
        loadZones(cfg_path);

        // QoS 与 tf
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "zones_markers", 10);
        zone_info_pub_ = this->create_publisher<interfaces::msg::ZoneInfo>(
            "zone_info", 10);

        // 定时更新(判断区域 + 发布可视化)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&ZoneManagerNode::update, this));
    }

   private:
    void loadZones(const std::string& path) {
        if (path.empty()) {
            RCLCPP_WARN(get_logger(), "zones_config_path 为空, 无区域加载");
            return;
        }
        try {
            YAML::Node root = YAML::LoadFile(path);
            YAML::Node zones_node = root["zones"];
            if (!zones_node || !zones_node.IsSequence()) {
                RCLCPP_WARN(get_logger(), "zones.yaml 缺少 zones 列表");
                return;
            }
            for (const auto& z : zones_node) {
                Zone zone;
                zone.name = z["name"].as<std::string>();
                zone.shape = z["shape"] ? z["shape"].as<std::string>() : "rectangle";

                if (zone.shape == "rectangle") {
                    auto center = z["center"].as<std::vector<double>>();
                    auto size = z["size"].as<std::vector<double>>();
                    zone.center_x = center[0];
                    zone.center_y = center[1];
                    zone.yaw = z["yaw"] ? z["yaw"].as<double>() : 0.0;
                    double hw = size[0] / 2.0, hh = size[1] / 2.0;
                    double cos_y = std::cos(zone.yaw), sin_y = std::sin(zone.yaw);
                    // 矩形四个角(世界坐标, 支持旋转)
                    double dx[4] = {-hw, hw, hw, -hw};
                    double dy[4] = {-hh, -hh, hh, hh};
                    for (int i = 0; i < 4; ++i) {
                        geometry_msgs::msg::Point p;
                        p.x = zone.center_x + dx[i] * cos_y - dy[i] * sin_y;
                        p.y = zone.center_y + dx[i] * sin_y + dy[i] * cos_y;
                        p.z = 0.0;
                        zone.points.push_back(p);
                    }
                } else if (zone.shape == "polygon") {
                    auto pts = z["points"].as<std::vector<std::vector<double>>>();
                    for (const auto& pt : pts) {
                        geometry_msgs::msg::Point p;
                        p.x = pt[0];
                        p.y = pt[1];
                        p.z = 0.0;
                        zone.points.push_back(p);
                        zone.center_x += p.x;
                        zone.center_y += p.y;
                    }
                    if (!zone.points.empty()) {
                        zone.center_x /= zone.points.size();
                        zone.center_y /= zone.points.size();
                    }
                }

                if (z["tasks"]) {
                    auto& t = z["tasks"];
                    if (t["spin"]) zone.tasks.spin_enabled = t["spin"].as<bool>();
                    if (t["face_hole"]) zone.tasks.face_hole = t["face_hole"].as<bool>();
                    if (t["nav_yaw"]) zone.tasks.nav_yaw = t["nav_yaw"].as<double>();
                    if (t["speed_limit"])
                        zone.tasks.speed_limit = t["speed_limit"].as<double>();
                    if (t["stop_distance"])
                        zone.tasks.stop_distance = t["stop_distance"].as<double>();
                }
                zones_.push_back(zone);
                RCLCPP_INFO(get_logger(), "加载区域: %s (shape=%s, center=(%.2f,%.2f))",
                            zone.name.c_str(), zone.shape.c_str(), zone.center_x,
                            zone.center_y);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "加载 zones.yaml 失败: %s", e.what());
        }
    }

    // 点在多边形内 (射线法)
    bool pointInPolygon(double px, double py, const std::vector<geometry_msgs::msg::Point>& poly) {
        bool inside = false;
        size_t n = poly.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            double xi = poly[i].x, yi = poly[i].y;
            double xj = poly[j].x, yj = poly[j].y;
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    bool getRobotPose(double& x, double& y, double& yaw) {
        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "获取机器人位姿失败: %s", e.what());
            return false;
        }
    }

    void publishMarkers() {
        visualization_msgs::msg::MarkerArray arr;
        for (size_t i = 0; i < zones_.size(); ++i) {
            const auto& zone = zones_[i];
            // 区域多边形
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = now();
            m.ns = "zones";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.06;  // 线宽
            m.color.r = zone.tasks.face_hole ? 1.0f : 0.2f;
            m.color.g = zone.tasks.spin_enabled ? 0.2f : 1.0f;
            m.color.b = 0.3f;
            m.color.a = 1.0f;
            m.pose.orientation.w = 1.0;
            for (const auto& p : zone.points) m.points.push_back(p);
            m.points.push_back(zone.points.front());  // 闭合
            arr.markers.push_back(m);

            // 半透明填充
            visualization_msgs::msg::Marker fill;
            fill.header = m.header;
            fill.ns = "zones_fill";
            fill.id = static_cast<int>(i);
            fill.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            fill.action = visualization_msgs::msg::Marker::ADD;
            fill.scale.x = 1.0;
            fill.scale.y = 1.0;
            fill.color.r = m.color.r;
            fill.color.g = m.color.g;
            fill.color.b = m.color.b;
            fill.color.a = 0.25f;
            fill.pose.orientation.w = 1.0;
            // 简单三角剖分: 以中心为顶点的扇形
            for (size_t k = 0; k < zone.points.size(); ++k) {
                geometry_msgs::msg::Point c;
                c.x = zone.center_x;
                c.y = zone.center_y;
                c.z = 0.0;
                fill.points.push_back(c);
                fill.points.push_back(zone.points[k]);
                fill.points.push_back(zone.points[(k + 1) % zone.points.size()]);
            }
            arr.markers.push_back(fill);

            // 文字标签
            visualization_msgs::msg::Marker txt;
            txt.header = m.header;
            txt.ns = "zones_label";
            txt.id = static_cast<int>(i);
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.scale.z = 0.5;
            txt.color.r = 1.0f;
            txt.color.g = 1.0f;
            txt.color.b = 1.0f;
            txt.color.a = 1.0f;
            txt.pose.position.x = zone.center_x;
            txt.pose.position.y = zone.center_y;
            txt.pose.position.z = 1.0;
            txt.pose.orientation.w = 1.0;
            txt.text = zone.name;
            arr.markers.push_back(txt);
        }
        marker_pub_->publish(arr);
    }

    void update() {
        publishMarkers();

        double x, y, yaw;
        if (!getRobotPose(x, y, yaw)) return;

        interfaces::msg::ZoneInfo info;
        info.in_zone = false;
        info.zone_name = "";
        info.nav_yaw = 0.0f;
        info.spin_enabled = true;
        info.speed_limit = 0.0f;
        info.zone_center_x = 0.0f;
        info.zone_center_y = 0.0f;

        for (const auto& zone : zones_) {
            if (pointInPolygon(x, y, zone.points)) {
                info.in_zone = true;
                info.zone_name = zone.name;
                info.nav_yaw = static_cast<float>(zone.tasks.nav_yaw);
                info.spin_enabled = zone.tasks.spin_enabled;
                info.speed_limit = static_cast<float>(zone.tasks.speed_limit);
                info.zone_center_x = static_cast<float>(zone.center_x);
                info.zone_center_y = static_cast<float>(zone.center_y);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "进入区域: %s", zone.name.c_str());
                break;
            }
        }
        zone_info_pub_->publish(info);
    }

    std::vector<Zone> zones_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<interfaces::msg::ZoneInfo>::SharedPtr zone_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace zone_mgr

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<zone_mgr::ZoneManagerNode>());
    rclcpp::shutdown();
    return 0;
}
