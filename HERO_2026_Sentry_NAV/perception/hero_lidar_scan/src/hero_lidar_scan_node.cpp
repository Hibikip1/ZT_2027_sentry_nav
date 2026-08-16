#include "hero_lidar_scan/hero_lidar_scan_node.hpp"
#ifdef OMP_LIDAR
#include <omp.h>
#else
#include <tbb/parallel_for.h>
#endif
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <pcl/impl/point_types.hpp>

namespace hero_lidar_scan {

HeroLidarScanNode::HeroLidarScanNode(const rclcpp::NodeOptions& options)
    : Node("hero_lidar_scan_node", options), max_odom_buffer_size_(1000) {
    // 声明参数
    this->declare_parameter<std::string>("lidar1_topic", "/livox/lidar1");
    this->declare_parameter<std::string>("lidar2_topic", "/livox/lidar2");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("fused_cloud_topic", "/fused_cloud");

    // 雷达1外参
    this->declare_parameter<double>("lidar1_x", 0.0);
    this->declare_parameter<double>("lidar1_y", 0.0);
    this->declare_parameter<double>("lidar1_z", 0.0);
    this->declare_parameter<double>("lidar1_roll", 0.0);
    this->declare_parameter<double>("lidar1_pitch", 0.0);
    this->declare_parameter<double>("lidar1_yaw", 0.0);

    // 雷达2外参
    this->declare_parameter<double>("lidar2_x", 0.0);
    this->declare_parameter<double>("lidar2_y", 0.0);
    this->declare_parameter<double>("lidar2_z", 0.0);
    this->declare_parameter<double>("lidar2_roll", 0.0);
    this->declare_parameter<double>("lidar2_pitch", 0.0);
    this->declare_parameter<double>("lidar2_yaw", 0.0);

    this->declare_parameter<double>("time_offset", 0.0);
    this->declare_parameter<double>("filter_size", 0.05);

    // 读取参数
    auto lidar1_topic = this->get_parameter("lidar1_topic").as_string();
    auto lidar2_topic = this->get_parameter("lidar2_topic").as_string();
    auto odom_topic = this->get_parameter("odom_topic").as_string();
    auto fused_cloud_topic =
        this->get_parameter("fused_cloud_topic").as_string();

    // 构建雷达1外参矩阵
    double l1_x = this->get_parameter("lidar1_x").as_double();
    double l1_y = this->get_parameter("lidar1_y").as_double();
    double l1_z = this->get_parameter("lidar1_z").as_double();
    double l1_roll = this->get_parameter("lidar1_roll").as_double();
    double l1_pitch = this->get_parameter("lidar1_pitch").as_double();
    double l1_yaw = this->get_parameter("lidar1_yaw").as_double();

    Eigen::AngleAxisd rollAngle1(l1_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle1(l1_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle1(l1_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q1 = yawAngle1 * pitchAngle1 * rollAngle1;
    T_lidar1_to_base_ = Eigen::Matrix4d::Identity();
    T_lidar1_to_base_.block<3, 3>(0, 0) = q1.toRotationMatrix();
    T_lidar1_to_base_(0, 3) = l1_x;
    T_lidar1_to_base_(1, 3) = l1_y;
    T_lidar1_to_base_(2, 3) = l1_z;

    // 构建雷达2外参矩阵
    double l2_x = this->get_parameter("lidar2_x").as_double();
    double l2_y = this->get_parameter("lidar2_y").as_double();
    double l2_z = this->get_parameter("lidar2_z").as_double();
    double l2_roll = this->get_parameter("lidar2_roll").as_double();
    double l2_pitch = this->get_parameter("lidar2_pitch").as_double();
    double l2_yaw = this->get_parameter("lidar2_yaw").as_double();

    Eigen::AngleAxisd rollAngle2(l2_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle2(l2_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle2(l2_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q2 = yawAngle2 * pitchAngle2 * rollAngle2;
    T_lidar2_to_base_ = Eigen::Matrix4d::Identity();
    T_lidar2_to_base_.block<3, 3>(0, 0) = q2.toRotationMatrix();
    T_lidar2_to_base_(0, 3) = l2_x;
    T_lidar2_to_base_(1, 3) = l2_y;
    T_lidar2_to_base_(2, 3) = l2_z;

    time_offset_ = this->get_parameter("time_offset").as_double();
    filter_size_ = this->get_parameter("filter_size").as_double();

    // 创建订阅器和发布器
    rclcpp::QoS qos(500);
    qos.best_effort();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos,
        std::bind(&HeroLidarScanNode::odomCallback, this,
                  std::placeholders::_1));

    auto lidar_qos = rclcpp::QoS(10)
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                         .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
#ifdef TWO_LIDARS
    lidar1_sub_.subscribe(this, lidar1_topic, lidar_qos.get_rmw_qos_profile());
    lidar2_sub_.subscribe(this, lidar2_topic, lidar_qos.get_rmw_qos_profile());
#else
    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar1_topic, lidar_qos,
        std::bind(&HeroLidarScanNode::lidarCallback, this,
                  std::placeholders::_1));
#endif
    ///////////////////////////////
    crop_filter_.setMin(min_pt);
    crop_filter_.setMax(max_pt);
    // setNegative(true) 表示去除 box 内的点，保留外部点
    crop_filter_.setNegative(true);
    // 在构造函数中初始化
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
#ifdef TWO_LIDARS
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), lidar1_sub_, lidar2_sub_);
    sync_->registerCallback(std::bind(&HeroLidarScanNode::lidarSyncCallback,
                                      this, std::placeholders::_1,
                                      std::placeholders::_2));
#endif
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        fused_cloud_topic, 10);

    worker_running_ = true;
    worker_thread_ = std::thread(&HeroLidarScanNode::workerLoop, this);
#ifdef OMP_LIDAR
    int max_threads = omp_get_max_threads();
    int limited_threads = std::max(1, max_threads - 6);
    omp_set_num_threads(limited_threads);
    RCLCPP_INFO(this->get_logger(),
                "HeroLidarScan节点已启动,time_offset:%f, OMP threads: %d->%d",
                time_offset_, max_threads, limited_threads);
#endif
}

HeroLidarScanNode::~HeroLidarScanNode() {
    worker_running_ = false;
    frame_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

// 里程计回调：维护时间有序队列
void HeroLidarScanNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push_back(*msg);
    if (odom_buffer_.size() > max_odom_buffer_size_) {
        odom_buffer_.pop_front();
    }
}

// 位姿插值查询：基于传入的只读本地里程计队列。线性插值平移，SLERP插值旋转
bool HeroLidarScanNode::getPoseAtTimeLocal(
    const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
    const double target_time, Eigen::Matrix4d& pose) {
    if (local_odom_buffer.size() < 2) return false;

    // 查找前后两帧
    size_t idx_after = 0;
    for (size_t i = 0; i < local_odom_buffer.size(); ++i) {
        if (rclcpp::Time(local_odom_buffer[i].header.stamp).seconds() >=
            target_time) {
            idx_after = i;
            break;
        }
    }

    if (idx_after == 0) {
        // 目标时间早于队列最早时间，使用最早帧
        auto& odom = local_odom_buffer.back();
        Eigen::Quaterniond q(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        pose(0, 3) = odom.pose.pose.position.x;
        pose(1, 3) = odom.pose.pose.position.y;
        pose(2, 3) = odom.pose.pose.position.z;
        return true;
    }

    size_t idx_before = idx_after - 1;
    auto& odom_before = local_odom_buffer[idx_before];
    auto& odom_after = local_odom_buffer[idx_after];

    double t_before = rclcpp::Time(odom_before.header.stamp).seconds();
    double t_after = rclcpp::Time(odom_after.header.stamp).seconds();
    double ratio = (target_time - t_before) / (t_after - t_before + 1e-9);
    ratio = std::clamp(ratio, 0.0, 1.0);

    // 平移线性插值
    Eigen::Vector3d pos_before(odom_before.pose.pose.position.x,
                               odom_before.pose.pose.position.y,
                               odom_before.pose.pose.position.z);
    Eigen::Vector3d pos_after(odom_after.pose.pose.position.x,
                              odom_after.pose.pose.position.y,
                              odom_after.pose.pose.position.z);
    Eigen::Vector3d pos_interp = pos_before + ratio * (pos_after - pos_before);

    // 旋转SLERP插值
    Eigen::Quaterniond q_before(odom_before.pose.pose.orientation.w,
                                odom_before.pose.pose.orientation.x,
                                odom_before.pose.pose.orientation.y,
                                odom_before.pose.pose.orientation.z);
    Eigen::Quaterniond q_after(
        odom_after.pose.pose.orientation.w, odom_after.pose.pose.orientation.x,
        odom_after.pose.pose.orientation.y, odom_after.pose.pose.orientation.z);
    Eigen::Quaterniond q_interp = q_before.slerp(ratio, q_after);

    pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
    pose.block<3, 1>(0, 3) = pos_interp;
    return true;
}

// 处理单个雷达点云去畸变：引入降频桶优化与无锁本地化查询
#ifdef OMP_LIDAR
void HeroLidarScanNode::processLidar(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
    const Eigen::Matrix4d& T_ext, const Eigen::Matrix4d& pose_end,
    const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
    pcl::PointCloud<pcl::PointXYZI>& cloud_out) {
    if (local_odom_buffer.empty()) {
        cloud_out.clear();
        return;
    }

    double timebase = msg->timebase * 1e-9;  // 纳秒转秒
    double max_odom_time =
        rclcpp::Time(local_odom_buffer.back().header.stamp).seconds();

    size_t valid_point_num = 0;
    for (size_t i = 0; i < msg->point_num; ++i) {
        double pt_time =
            timebase + msg->points[i].offset_time * 1e-9 + time_offset_;
        if (pt_time > max_odom_time) {
            break;
        }
        valid_point_num++;
    }

    if (valid_point_num == 0) {
        cloud_out.clear();
        return;
    }

    cloud_out.points.resize(valid_point_num);

    // 动态桶策略 (分段预积分)：假设每 5 毫秒 (0.005 秒)
    // 视作一个固定位姿桶，大幅减少插值次数
    const double BUCKET_TIME_RESOLUTION = 0.0025;

    // 计算每个桶的位姿并缓存
    double total_duration = msg->points[valid_point_num - 1].offset_time * 1e-9;

    size_t num_buckets =
        static_cast<size_t>(total_duration / BUCKET_TIME_RESOLUTION) + 1;
    std::vector<Eigen::Matrix4d> bucket_transforms(num_buckets);
    std::vector<bool> bucket_valid(num_buckets, false);

// 这里的桶大小大概也就 10-20 个，顺带并行处理
#pragma omp parallel for
    for (size_t b = 0; b < num_buckets; ++b) {
        double bucket_time =
            timebase + b * BUCKET_TIME_RESOLUTION + time_offset_;
        Eigen::Matrix4d pose_i;
        if (getPoseAtTimeLocal(local_odom_buffer, bucket_time, pose_i)) {
            // 预计算出当前桶的转换矩阵: pose_i * T_ext
            bucket_transforms[b] = pose_i * T_ext;
            bucket_valid[b] = true;
        }
    }

// 遍历点云并应用桶矩阵变换
#pragma omp parallel for
    for (size_t i = 0; i < valid_point_num; ++i) {
        const auto& pt = msg->points[i];
        double pt_offset_time = pt.offset_time * 1e-9;

        // 找出该点对应的桶索引
        size_t bucket_idx =
            static_cast<size_t>(pt_offset_time / BUCKET_TIME_RESOLUTION);
        if (bucket_idx >= num_buckets) bucket_idx = num_buckets - 1;

        // 如果该桶未能拿到位姿（由于上面截断，所以绝大部分应当命中有效位姿）
        Eigen::Matrix4d transform;
        if (bucket_valid[bucket_idx]) {
            transform = bucket_transforms[bucket_idx];
        } else {
            transform = T_ext;
        }

        Eigen::Vector4d p_raw(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d p_undist = transform * p_raw;

        cloud_out.points[i].x = p_undist(0);
        cloud_out.points[i].y = p_undist(1);
        cloud_out.points[i].z = p_undist(2);
        cloud_out.points[i].intensity = pt.reflectivity;
    }
}
#else
// TBB
void HeroLidarScanNode::processLidar(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
    const Eigen::Matrix4d& T_ext, const Eigen::Matrix4d& pose_end,
    const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
    pcl::PointCloud<pcl::PointXYZI>& cloud_out) {
    if (local_odom_buffer.empty()) {
        cloud_out.clear();
        return;
    }

    double timebase = msg->timebase * 1e-9;  // 纳秒转秒
    double max_odom_time =
        rclcpp::Time(local_odom_buffer.back().header.stamp).seconds();

    size_t valid_point_num = 0;
    for (size_t i = 0; i < msg->point_num; ++i) {
        double pt_time =
            timebase + msg->points[i].offset_time * 1e-9 + time_offset_;
        if (pt_time > max_odom_time) {
            break;
        }
        valid_point_num++;
    }

    if (valid_point_num == 0) {
        cloud_out.clear();
        return;
    }

    cloud_out.points.resize(valid_point_num);

    // 动态桶策略 (分段预积分)：假设每 5 毫秒 (0.005 秒)
    // 视作一个固定位姿桶，大幅减少插值次数
    const double BUCKET_TIME_RESOLUTION = 0.0025;

    // 计算每个桶的位姿并缓存
    double total_duration = msg->points[valid_point_num - 1].offset_time * 1e-9;

    size_t num_buckets =
        static_cast<size_t>(total_duration / BUCKET_TIME_RESOLUTION) + 1;
    std::vector<Eigen::Matrix4d> bucket_transforms(num_buckets);
    std::vector<bool> bucket_valid(num_buckets, false);

    // 这里的桶大小大概也就 10-20 个，顺带并行处理
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, num_buckets),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t b = range.begin(); b != range.end(); ++b) {
                double bucket_time =
                    timebase + b * BUCKET_TIME_RESOLUTION + time_offset_;
                Eigen::Matrix4d pose_i;
                if (getPoseAtTimeLocal(local_odom_buffer, bucket_time,
                                       pose_i)) {
                    bucket_transforms[b] = pose_i * T_ext;
                    bucket_valid[b] = true;
                }
            }
        });

    // 遍历点云并应用桶矩阵变换
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, valid_point_num),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                const auto& pt = msg->points[i];
                double pt_offset_time = pt.offset_time * 1e-9;

                size_t bucket_idx = static_cast<size_t>(pt_offset_time /
                                                        BUCKET_TIME_RESOLUTION);
                if (bucket_idx >= num_buckets) bucket_idx = num_buckets - 1;

                Eigen::Matrix4d transform;
                // if (bucket_valid[bucket_idx]) {
                    transform = bucket_transforms[bucket_idx];
                // } else {
                //     transform = T_ext;
                //     std::cout << "WARNING" <<std::endl;
                // }

                Eigen::Vector4d p_raw(pt.x, pt.y, pt.z, 1.0);
                Eigen::Vector4d p_undist = transform * p_raw;

                cloud_out.points[i].x = p_undist(0);
                cloud_out.points[i].y = p_undist(1);
                cloud_out.points[i].z = p_undist(2);
                cloud_out.points[i].intensity = pt.reflectivity;
            }
        });
}

#endif
#ifdef TWO_LIDARS
// 双雷达同步回调：仅做数据快照入队，不做任何计算
void HeroLidarScanNode::lidarSyncCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar1_msg,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar2_msg) {
    std::deque<nav_msgs::msg::Odometry> local_odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "里程计队列为空");
            return;
        }
        local_odom = odom_buffer_;
    }
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex_);
        if (frame_queue_.size() >= 3) {
            frame_queue_.pop();
        }
        frame_queue_.push({lidar1_msg, lidar2_msg, std::move(local_odom)});
    }
    frame_cv_.notify_one();
}
#else
// 单雷达回调：仅做数据快照入队，不做任何计算
void HeroLidarScanNode::lidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_msg) {
    std::deque<nav_msgs::msg::Odometry> local_odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "里程计队列为空");
            return;
        }
        local_odom = odom_buffer_;
    }
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex_);
        if (frame_queue_.size() >= 3) {
            frame_queue_.pop();
        }
        frame_queue_.push({lidar_msg, std::move(local_odom)});
    }
    frame_cv_.notify_one();
}
#endif
// 工作线程：执行所有 OMP 计算和点云发布
void HeroLidarScanNode::workerLoop() {
    while (worker_running_) {
#ifdef TWO_LIDARS
        LidarFrame frame;
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            frame_cv_.wait(lock, [this] {
                return !frame_queue_.empty() || !worker_running_;
            });
            if (!worker_running_ && frame_queue_.empty()) break;
            frame = std::move(frame_queue_.front());
            frame_queue_.pop();
        }

        auto& lidar1_msg = frame.lidar1;
        auto& lidar2_msg = frame.lidar2;
        auto& local_odom_buffer = frame.odom_snapshot;

        // 1. 计算时间范围
        double t1_end = lidar1_msg->timebase * 1e-9;
        double t2_end = lidar2_msg->timebase * 1e-9;
        if (!lidar1_msg->points.empty()) {
            t1_end += lidar1_msg->points.back().offset_time * 1e-9;
        }
        if (!lidar2_msg->points.empty()) {
            t2_end += lidar2_msg->points.back().offset_time * 1e-9;
        }
        double t_end = std::max(t1_end, t2_end);
        rclcpp::Time stamp_end(static_cast<int64_t>(t_end * 1e9));

        // 2. 查询基准时刻位姿
        Eigen::Matrix4d pose_end;
        if (!getPoseAtTimeLocal(local_odom_buffer, t_end, pose_end)) {
            RCLCPP_WARN(this->get_logger(), "无法获取基准时刻位姿");
            continue;
        }

        // 3. 处理两个雷达点云
        pcl::PointCloud<pcl::PointXYZI> cloud1, cloud2;
        processLidar(lidar1_msg, T_lidar1_to_base_, pose_end, local_odom_buffer,
                     cloud1);
        processLidar(lidar2_msg, T_lidar2_to_base_, pose_end, local_odom_buffer,
                     cloud2);

        // 融合点云
        pcl::PointCloud<pcl::PointXYZI> fused_cloud = cloud1 + cloud2;
#else
        LidarFrame frame;
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            frame_cv_.wait(lock, [this] {
                return !frame_queue_.empty() || !worker_running_;
            });
            if (!worker_running_ && frame_queue_.empty()) break;
            frame = std::move(frame_queue_.front());
            frame_queue_.pop();
        }
        auto& lidar_msg = frame.lidar;
        auto& local_odom_buffer = frame.odom_snapshot;
        // 1. 计算时间范围
        double t_end = lidar_msg->timebase * 1e-9;
        if (!lidar_msg->points.empty()) {
            t_end += lidar_msg->points.back().offset_time * 1e-9;
        }
        rclcpp::Time stamp_end(static_cast<int64_t>(t_end * 1e9));
        // 2. 查询基准时刻位姿
        Eigen::Matrix4d pose_end;
        if (!getPoseAtTimeLocal(local_odom_buffer, t_end, pose_end)) {
            RCLCPP_WARN(this->get_logger(), "无法获取基准时刻位姿");
            continue;
        }
        // 3. 处理雷达点云
        pcl::PointCloud<pcl::PointXYZI> fused_cloud;
        processLidar(lidar_msg, T_lidar2_to_base_, pose_end, local_odom_buffer,
                     fused_cloud);

#endif
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
        auto fused_ptr =
            std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(fused_cloud);
        crop_filter_.setInputCloud(fused_ptr);
        crop_filter_.filter(filtered_cloud);

        pcl::PointCloud<pcl::PointXYZI> voxel_filtered_cloud;
        if (0) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            auto crop_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
                filtered_cloud);
            voxel_filter.setInputCloud(crop_ptr);
            voxel_filter.setLeafSize(filter_size_, filter_size_, filter_size_);
            voxel_filter.filter(voxel_filtered_cloud);
        } else {
            voxel_filtered_cloud = filtered_cloud;
        }
        // 发布点云
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(voxel_filtered_cloud, output_msg);
        output_msg.header.stamp = lidar_msg->header.stamp;  // 保持原雷达时间戳
        output_msg.header.frame_id = "odom";  // 保持原雷达坐标系
        cloud_pub_->publish(output_msg);
    }
}

}  // namespace hero_lidar_scan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hero_lidar_scan::HeroLidarScanNode)
