#ifndef HERO_LIDAR_SCAN_NODE_HPP_
#define HERO_LIDAR_SCAN_NODE_HPP_
#ifdef TWO_LIDARS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#endif
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace hero_lidar_scan {
const float crop_min_x = -0.6f;
const float crop_min_y = -0.6f;
const float crop_min_z = -0.3f;
const float crop_max_x = 0.6f;
const float crop_max_y = 0.6f;
const float crop_max_z = 1.0f;
class HeroLidarScanNode : public rclcpp::Node {
   public:
    explicit HeroLidarScanNode(const rclcpp::NodeOptions& options);
    ~HeroLidarScanNode();

   private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    #ifdef TWO_LIDARS
    void lidarSyncCallback(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar1_msg,
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar2_msg);
    #else
        void lidarCallback(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_msg);
    #endif
    bool getPoseAtTimeLocal(
        const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
        double target_time, Eigen::Matrix4d& pose);

    void processLidar(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
        const Eigen::Matrix4d& T_ext, const Eigen::Matrix4d& pose_end,
        const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
        pcl::PointCloud<pcl::PointXYZI>& cloud_out);

    void workerLoop();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    #ifdef TWO_LIDARS
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar1_sub_;
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar2_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
#else 
rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
#endif   
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    std::deque<nav_msgs::msg::Odometry> odom_buffer_;
    std::mutex odom_mutex_;
    size_t max_odom_buffer_size_;
    pcl::CropBox<pcl::PointXYZI> crop_filter_;

    Eigen::Vector4f min_pt{crop_min_x, crop_min_y, crop_min_z, 1.0f};
    Eigen::Vector4f max_pt{crop_max_x, crop_max_y, crop_max_z, 1.0f};

    Eigen::Matrix4d T_lidar1_to_base_;
    Eigen::Matrix4d T_lidar2_to_base_; 
std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double time_offset_;
    double filter_size_;
#ifdef TWO_LIDARS
    struct LidarFrame {
        livox_ros_driver2::msg::CustomMsg::ConstSharedPtr lidar1;
        livox_ros_driver2::msg::CustomMsg::ConstSharedPtr lidar2;
        std::deque<nav_msgs::msg::Odometry> odom_snapshot;
    };
#else 
 struct LidarFrame {
        livox_ros_driver2::msg::CustomMsg::ConstSharedPtr lidar;
        std::deque<nav_msgs::msg::Odometry> odom_snapshot;
    };
#endif
    std::queue<LidarFrame> frame_queue_;
    std::mutex frame_queue_mutex_;
    std::condition_variable frame_cv_;
    std::thread worker_thread_;
    std::atomic<bool> worker_running_{false};
};

}  // namespace hero_lidar_scan

#endif  // HERO_LIDAR_SCAN_NODE_HPP_
