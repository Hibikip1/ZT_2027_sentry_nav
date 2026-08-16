#include "lidar_merge/merge.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#ifdef DEBUG
#include <visualization_msgs/msg/marker.hpp>
#endif
namespace lidar_merge
{
LidarMergeNode::LidarMergeNode(const std::string & name) : Node(name)
{
  this->declare_parameter("lidar1_topic", "/livox/lidar1");
#ifdef DEBUG
  ori_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/livox/ori_marker", 1);
#endif
  this->declare_parameter("lidar2_topic", "/livox/lidar2");
  this->declare_parameter("lidar1_to_odin1_x", 0.0);
  this->declare_parameter("lidar1_to_odin1_y", 0.0);
  this->declare_parameter("lidar1_to_odin1_z", 0.0);
  this->declare_parameter("lidar1_to_odin1_roll", 0.0);
  this->declare_parameter("lidar1_to_odin1_pitch", 0.0);
  this->declare_parameter("lidar1_to_odin1_yaw", 0.0);
  
  this->declare_parameter("lidar2_to_odin1_x", 0.0);
  this->declare_parameter("lidar2_to_odin1_y", 0.0);
  this->declare_parameter("lidar2_to_odin1_z", 0.0);
  this->declare_parameter("lidar2_to_odin1_roll", 0.0);
  this->declare_parameter("lidar2_to_odin1_pitch", 0.0);
  this->declare_parameter("lidar2_to_odin1_yaw", 0.0);

  this->declare_parameter("blind_distance", 0.25);

  this->declare_parameter("ori_x", -0.2);
  this->declare_parameter("ori_y", 0.0);
  this->declare_parameter("ori_z", -0.1);
   ori_x = this->get_parameter("ori_x").as_double();
   ori_y = this->get_parameter("ori_y").as_double();
   ori_z = this->get_parameter("ori_z").as_double();
  this->get_parameter("blind_distance", blind);
  auto lidar1_topic = this->get_parameter("lidar1_topic").as_string();
  auto lidar2_topic = this->get_parameter("lidar2_topic").as_string();

  double l1_to_odin1_x = this->get_parameter("lidar1_to_odin1_x").as_double();
  double l1_to_odin1_y = this->get_parameter("lidar1_to_odin1_y").as_double();
  double l1_to_odin1_z = this->get_parameter("lidar1_to_odin1_z").as_double();
  double l1_to_odin1_roll = this->get_parameter("lidar1_to_odin1_roll").as_double();
  double l1_to_odin1_pitch = this->get_parameter("lidar1_to_odin1_pitch").as_double();
  double l1_to_odin1_yaw = this->get_parameter("lidar1_to_odin1_yaw").as_double();

  double l2_to_odin1_x = this->get_parameter("lidar2_to_odin1_x").as_double();
  double l2_to_odin1_y = this->get_parameter("lidar2_to_odin1_y").as_double();
  double l2_to_odin1_z = this->get_parameter("lidar2_to_odin1_z").as_double();
  double l2_to_odin1_roll = this->get_parameter("lidar2_to_odin1_roll").as_double();
  double l2_to_odin1_pitch = this->get_parameter("lidar2_to_odin1_pitch").as_double();
  double l2_to_odin1_yaw = this->get_parameter("lidar2_to_odin1_yaw").as_double();

  Eigen::AngleAxisd rollAngle(l1_to_odin1_roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(l1_to_odin1_pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(l1_to_odin1_yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  T_lidar1_to_odin1_ = Eigen::Matrix4d::Identity();
  T_lidar1_to_odin1_.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_lidar1_to_odin1_(0, 3) = l1_to_odin1_x;
  T_lidar1_to_odin1_(1, 3) = l1_to_odin1_y;
  T_lidar1_to_odin1_(2, 3) = l1_to_odin1_z;
    lidar1r00 = T_lidar1_to_odin1_(0, 0); 
    lidar1r01 = T_lidar1_to_odin1_(0, 1);
    lidar1r02 = T_lidar1_to_odin1_(0, 2); 
    lidar1tx = T_lidar1_to_odin1_(0, 3);
    lidar1r10 = T_lidar1_to_odin1_(1, 0); 
    lidar1r11 = T_lidar1_to_odin1_(1, 1);
    lidar1r12 = T_lidar1_to_odin1_(1, 2); 
    lidar1ty = T_lidar1_to_odin1_(1, 3);
    lidar1r20 = T_lidar1_to_odin1_(2, 0); 
    lidar1r21 = T_lidar1_to_odin1_(2, 1);
    lidar1r22 = T_lidar1_to_odin1_(2, 2); 
    lidar1tz = T_lidar1_to_odin1_(2, 3);

    T_lidar2_to_odin1_ = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rollAngle2(l2_to_odin1_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle2(l2_to_odin1_pitch, Eigen::Vector3d::UnitY());
     Eigen::AngleAxisd yawAngle2(l2_to_odin1_yaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q2 = yawAngle2 * pitchAngle2 * rollAngle2;
    T_lidar2_to_odin1_.block<3, 3>(0, 0) = q2.toRotationMatrix();
    T_lidar2_to_odin1_(0, 3) = l2_to_odin1_x;
    T_lidar2_to_odin1_(1, 3) = l2_to_odin1_y;
    T_lidar2_to_odin1_(2, 3) = l2_to_odin1_z;
    lidar2r00 = T_lidar2_to_odin1_(0, 0); 
    lidar2r01 = T_lidar2_to_odin1_(0, 1);
    lidar2r02 = T_lidar2_to_odin1_(0, 2); 
    lidar2tx = T_lidar2_to_odin1_(0, 3);
    lidar2r10 = T_lidar2_to_odin1_(1, 0); 
    lidar2r11 = T_lidar2_to_odin1_(1, 1);
    lidar2r12 = T_lidar2_to_odin1_(1, 2); 
    lidar2ty = T_lidar2_to_odin1_(1, 3);
    lidar2r20 = T_lidar2_to_odin1_(2, 0); 
    lidar2r21 = T_lidar2_to_odin1_(2, 1);
    lidar2r22 = T_lidar2_to_odin1_(2, 2); 
    lidar2tz = T_lidar2_to_odin1_(2, 3);
  cloud_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/fused_cloud", 10);
  lidar1_sub_.subscribe(this, lidar1_topic);
  lidar2_sub_.subscribe(this, lidar2_topic);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(5), lidar1_sub_, lidar2_sub_);
  sync_->registerCallback(std::bind(
    &LidarMergeNode::Sync_lidarCallback, this, std::placeholders::_1, std::placeholders::_2));
}
// ...existing code...

bool LidarMergeNode::isBlind(const livox_ros_driver2::msg::CustomPoint & pt) {
  if(((pt.tag&0x30)!=0x10)&&((pt.tag&0x30)!=0x00)) return true; // tag不为0的点直接丢弃，认为是盲区点s
  // if((pt.reflectivity<30)&&((pt.tag&0x30)==0x10)) return true; // 强度为0的点直接丢弃，认为是盲区点
  // if(pt.reflectivity<5) return true;
  double distance = std::sqrt((pt.x - ori_x) * (pt.x - ori_x) + (pt.y - ori_y) * (pt.y - ori_y) + (pt.z - ori_z) * (pt.z - ori_z));
 return distance < blind;
} 
   
void LidarMergeNode::Sync_lidarCallback(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2)
{
  auto start_time =  std::chrono::steady_clock::now();

  auto fused_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>(); 
  
  // 1. 使用两个消息中最新(最大的) timebase 和 header
  const bool is_msg1_latest = (msg1->timebase > msg2->timebase);
  fused_msg->header = is_msg1_latest ? msg1->header : msg2->header;
  fused_msg->timebase = is_msg1_latest ? msg1->timebase : msg2->timebase;
  fused_msg->lidar_id = msg2->lidar_id;

  const size_t len1 = msg1->points.size();
  const size_t len2 = msg2->points.size();
  fused_msg->points.resize(len1 + len2);

  // 2. 分别计算相对于最新 timebase 的偏差 (结果必定 <= 0)
  const int64_t target_timebase = static_cast<int64_t>(fused_msg->timebase);
  const int64_t diff1 = static_cast<int64_t>(msg1->timebase) - target_timebase;
  const int64_t diff2 = static_cast<int64_t>(msg2->timebase) - target_timebase;

  // 用一个结构体暂存带 int64_t 的点，防止 uint32_t 溢出检测失效
  struct PointExt {
    livox_ros_driver2::msg::CustomPoint pt;
    int64_t real_offset;
  };
  std::vector<PointExt> trans_p1(len1);
  std::vector<livox_ros_driver2::msg::CustomPoint> trans_p2(len2);

  // 3. TBB 并行计算 msg1 的坐标变换
  tbb::parallel_for(tbb::blocked_range<size_t>(0, len1), [&](const tbb::blocked_range<size_t> & r) {
    for (size_t i = r.begin(); i != r.end(); ++i) {
      livox_ros_driver2::msg::CustomPoint pt = msg1->points[i];
      const double px = pt.x, py = pt.y, pz = pt.z;
      pt.x = static_cast<float>(lidar1r00 * px + lidar1r01 * py + lidar1r02 * pz + lidar1tx);
      pt.y = static_cast<float>(lidar1r10 * px + lidar1r11 * py + lidar1r12 * pz + lidar1ty);
      pt.z = static_cast<float>(lidar1r20 * px + lidar1r21 * py + lidar1r22 * pz + lidar1tz);

      trans_p1[i].real_offset = diff1 + static_cast<int64_t>(pt.offset_time);
      trans_p1[i].pt = pt;
    }
  });
  tbb::parallel_for(tbb::blocked_range<size_t>(0, len2), [&](const tbb::blocked_range<size_t> & r) {
    for (size_t i = r.begin(); i != r.end(); ++i) {
      livox_ros_driver2::msg::CustomPoint pt = msg2->points[i];
      const double px = pt.x, py = pt.y, pz = pt.z;
      pt.x = static_cast<float>(lidar2r00 * px + lidar2r01 * py + lidar2r02 * pz + lidar2tx);
      pt.y = static_cast<float>(lidar2r10 * px + lidar2r11 * py + lidar2r12 * pz + lidar2ty);
      pt.z = static_cast<float>(lidar2r20 * px + lidar2r21 * py + lidar2r22 * pz + lidar2tz);
      trans_p2[i] = pt;
    }
  });
  // 4. 双指针归并
  livox_ros_driver2::msg::CustomPoint * out_ptr = fused_msg->points.data();
  const livox_ros_driver2::msg::CustomPoint * p2_ptr = trans_p2.data();

  size_t i = 0, j = 0, out_idx = 0;

  // 跳过 msg1 中早于 target_timebase 的非法负时间点
  while (i < len1 && trans_p1[i].real_offset < 0) { i++; }
  // 跳过 msg2 中早于 target_timebase 的非法负时间点
  while (j < len2 && (diff2 + static_cast<int64_t>(p2_ptr[j].offset_time)) < 0) { j++; }

  while (i < len1 && j < len2) {
    if (isBlind(trans_p1[i].pt) ) {
      i++;
      continue;
    }
    if (isBlind(p2_ptr[j]) ) {
      j++;
      continue;
    }
      // 如果任一点在盲区内，则直接丢弃，跳过比较 时间戳的步骤，继续比较下一点
    int64_t t1 = trans_p1[i].real_offset;
    int64_t t2 = diff2 + static_cast<int64_t>(p2_ptr[j].offset_time);

    if (t1 <= t2) {
      out_ptr[out_idx] = trans_p1[i].pt;
      out_ptr[out_idx].offset_time = static_cast<uint32_t>(t1);
      out_idx++; i++;
    } else {
      out_ptr[out_idx] = p2_ptr[j];
      out_ptr[out_idx].offset_time = static_cast<uint32_t>(t2);
      out_idx++; j++;
    }
  }

  while (i < len1) {
    out_ptr[out_idx] = trans_p1[i].pt;
    out_ptr[out_idx].offset_time = static_cast<uint32_t>(trans_p1[i].real_offset);
    out_idx++; i++;
  }
  while (j < len2) {
    out_ptr[out_idx] = p2_ptr[j];
    out_ptr[out_idx].offset_time = static_cast<uint32_t>(diff2 + p2_ptr[j].offset_time);
    out_idx++; j++;
  }

  fused_msg->points.resize(out_idx);
  fused_msg->point_num = out_idx;

#ifdef DEBUG
  visualization_msgs::msg::Marker ori_marker;
  ori_marker.header.frame_id = "base_link";
  ori_marker.header.stamp = fused_msg->header.stamp;
  ori_marker.ns = "lidar_merge_ori";
  ori_marker.id = 0;
  ori_marker.type = visualization_msgs::msg::Marker::SPHERE;
  ori_marker.action = visualization_msgs::msg::Marker::ADD;
  const Eigen::Matrix4d T_lidar2_to_base = []() {
    const double lidar2_x = 0.0;
    const double lidar2_y = -0.217;
    const double lidar2_z = 0.121;
    const double lidar2_roll = 0.0;
    const double lidar2_pitch = 0.61086;
    const double lidar2_yaw = -1.57079;
    Eigen::AngleAxisd roll_angle(lidar2_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(lidar2_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(lidar2_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform(0, 3) = lidar2_x;
    transform(1, 3) = lidar2_y;
    transform(2, 3) = lidar2_z;
    return transform;
  }();
  const Eigen::Vector4d ori_lidar2(ori_x, ori_y, ori_z, 1.0);
  const Eigen::Vector4d ori_base = T_lidar2_to_base * ori_lidar2;
  ori_marker.pose.position.x = ori_base.x();
  ori_marker.pose.position.y = ori_base.y();
  ori_marker.pose.position.z = ori_base.z();
  const Eigen::Quaterniond q_base(T_lidar2_to_base.block<3, 3>(0, 0));
  ori_marker.pose.orientation.x = q_base.x();
  ori_marker.pose.orientation.y = q_base.y();
  ori_marker.pose.orientation.z = q_base.z();
  ori_marker.pose.orientation.w = q_base.w();
  ori_marker.scale.x = 0.15;
  ori_marker.scale.y = 0.15;
  ori_marker.scale.z = 0.15;
  ori_marker.color.r = 1.0f;
  ori_marker.color.g = 0.2f;
  ori_marker.color.b = 0.2f;
  ori_marker.color.a = 1.0f;
  // ori_marker.lifetime = rclcpp::Duration::from_seconds(0.2).to_msg();
  ori_marker_pub_->publish(ori_marker);
#endif
  std::cout<<out_idx<<std::endl;
  PublishPointCloud(std::move(fused_msg));

  auto end_time = std::chrono::steady_clock::now();
  double time_consuming =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
  std::cout << " -- [ INFO] Map updated in " << time_consuming << " ms." << std::endl;
}
// ...existing code...
void LidarMergeNode::PublishPointCloud(livox_ros_driver2::msg::CustomMsg::UniquePtr msg1)
{
  // 发布融合后的点云
  cloud_pub_->publish(std::move(msg1));
}
}  // namespace lidar_merge
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_merge::LidarMergeNode>("lidar_merge_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
