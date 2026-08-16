#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef DEBUG
#include <visualization_msgs/msg/marker.hpp>
#endif
namespace lidar_merge
{

class LidarMergeNode : public rclcpp::Node
{
public:
  LidarMergeNode(const std::string & name);
  ~LidarMergeNode() = default;

  void Sync_lidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2);
  void PublishPointCloud(livox_ros_driver2::msg::CustomMsg::UniquePtr msg1);
  bool isBlind(const livox_ros_driver2::msg::CustomPoint & pt);
private:
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
#ifdef DEBUG
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ori_marker_pub_;
#endif
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar1_sub_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar2_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  Eigen::Matrix4d T_lidar1_to_odin1_;
  Eigen::Matrix4d T_lidar2_to_odin1_;
  double lidar1r00, lidar1r01, lidar1r02, lidar1tx;
  double lidar1r10, lidar1r11, lidar1r12, lidar1ty;
  double lidar1r20, lidar1r21, lidar1r22, lidar1tz;
  
  double lidar2r00, lidar2r01, lidar2r02, lidar2tx;
  double lidar2r10, lidar2r11, lidar2r12, lidar2ty;
  double lidar2r20, lidar2r21, lidar2r22, lidar2tz;
  double ori_x = 0.0;
  double ori_y = 0.0;
  double ori_z = 0.0;
  double blind =0.25;
};
}  // namespace lidar_merge