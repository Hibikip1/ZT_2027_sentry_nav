#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef DEBUG
#include <visualization_msgs/msg/marker.hpp>
#endif
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
namespace lidar_merge
{

class LidarMergeNode : public rclcpp::Node
{
public:
  LidarMergeNode(const std::string & name);
  ~LidarMergeNode() = default;
#ifdef TWO_LIDARS
  void Sync_lidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2);
    #else 
    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg);
    #endif 
  void PublishPointCloud(livox_ros_driver2::msg::CustomMsg::UniquePtr msg1);
  bool isBlind(const livox_ros_driver2::msg::CustomPoint & pt);
private:  
#ifdef DEBUG
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ori_marker_pub_;
#endif

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  #ifdef TWO_LIDARS
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar1_sub_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar2_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
#else 
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
#endif
  Eigen::Matrix4d T_lidar1_to_lidar2_;
   double r00, r01, r02, tx;
  double r10, r11, r12, ty;
  double r20, r21, r22, tz;

  double ori_x;
  double ori_z;
  double ori_y;
  double blind;


};
}  // namespace lidar_merge