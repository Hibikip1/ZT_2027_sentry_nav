#include "dog_map/point_cloud_define.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/impl/point_types.hpp>
#ifdef ODIN_WIDTH_LIDAR
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#endif
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "dog_map/occ_map.hpp"
#include "pcl_ros/transforms.hpp"

using namespace analy_utils;
class DogMapNode : public rclcpp::Node {
   public:
    struct ROSCallback {
        rclcpp::CallbackGroup::SharedPtr  cloud_me_cbk_group,
            update_cbk_group;
#ifndef ODIN_WIDTH_LIDAR 
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            cloud_sub;
#else 
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_sync;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> odin_sub_sync;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
#endif
        int unfinished_frame_cnt{0};
        Pose pc_pose;
        pcl::PointCloud<HEROPointXYZK> pc;
        rclcpp::TimerBase::SharedPtr update_timer;
        std::mutex update_lock;
    } rc_;
    struct CostPub {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_pc_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
            ground_pc_pub;
#ifdef FIX_MAP
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fix_pc_pub;
#endif
    } cost_pub_;
    struct RobotState {
        Vec3f p, v, a, j;
        double yaw;
        double rcv_time;
        bool rcv{false};
        Quatf q;
    } robot_state_;

  
#ifndef ODIN_WIDTH_LIDAR    
    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

        pcl::PointCloud<pcl::PointXYZ> temp_cloud_in;
        pcl::fromROSMsg(*cloud_msg, temp_cloud_in);
        cloud_in_->clear();
        cloud_in_->points.reserve(temp_cloud_in.points.size());
         for (auto& pt : temp_cloud_in.points) {
            cloud_in_->points.emplace_back(pt.x, pt.y, pt.z);
        }
        Eigen::Matrix4f tf_mat = Eigen::Matrix4f::Identity();
        try {
            tf_stamped = this->tf_buffer_->lookupTransform(
                "map", cloud_msg->header.frame_id, tf2::TimePointZero);
            tf_stamped.transform.translation.z += tf_z_offset_;  // z 偏移,防越界(参数化)
            const auto& tr = tf_stamped.transform.translation;
            const auto& rr = tf_stamped.transform.rotation;

            Eigen::Quaternionf q(
                static_cast<float>(rr.w), static_cast<float>(rr.x),
                static_cast<float>(rr.y), static_cast<float>(rr.z));
            q.normalize();

            tf_mat.setIdentity();
            tf_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
            tf_mat(0, 3) = static_cast<float>(tr.x);
            tf_mat(1, 3) = static_cast<float>(tr.y);
            tf_mat(2, 3) = static_cast<float>(tr.z);
            tf_stamped = this->tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);  // pb2025 机器人无 base_link 帧,用 base_footprint

        } catch (const std::exception & e) {
            std::cout << " -- [DOG] TF lookup failed (map -> "
                      << cloud_msg->header.frame_id
                      << "), skip this frame: " << e.what() << std::endl;
            return;
        } catch (...) {
            std::cout << " -- [DOG] Cloud callback error (unknown), "
                         "skip this frame."
                      << std::endl;
            return;
        }
        // tf

        // pcl::PointCloud<pcl::PointXYZ> temp_pc;
        pcl::transformPointCloud(*cloud_in_, *cloud_in_, tf_mat);
         if (!cloud_in_->empty()) {
            auto msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*cloud_in_, msg);

            msg.header.frame_id = "map";
            msg.header.stamp = this->get_clock()->now();
            cost_pub_.ground_pc_pub->publish(std::move(msg));
        }
        rc_.update_lock.lock();
        rc_.pc = *cloud_in_;
        {
             rc_.pc_pose.first[0] = tf_stamped.transform.translation.x;
            rc_.pc_pose.first[1] = tf_stamped.transform.translation.y;
            rc_.pc_pose.first[2] = tf_stamped.transform.translation.z;
           
        }
        
        rc_.unfinished_frame_cnt++;
        map_empty_ = false;
        rc_.update_lock.unlock();
//    std::cout<<"cloud finish .."<<std::endl;
    }
#else 
    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& odin_msg) {
         pcl::PointCloud<pcl::PointXYZ> temp_cloud_in_360;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_in_odin=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*cloud_msg, temp_cloud_in_360);
        pcl::fromROSMsg(*odin_msg, *temp_cloud_in_odin);
        pcl::PassThrough<pcl::PointXYZ> ps;
        ps.setFilterFieldName("z");
        ps.setInputCloud(temp_cloud_in_odin);
        ps.setFilterLimits(0.06, 1.0);
        ps.filter(*temp_cloud_in_odin);
        
        //  清理 cloud_in_ 并准确预计算最终容量（360完整点数 + odin全点数预测）
        cloud_in_->clear();
        cloud_in_->reserve(temp_cloud_in_360.points.size() + temp_cloud_in_odin->points.size());

        //  填充 360 雷达点云 
        for (const auto& pt : temp_cloud_in_360.points) {
            cloud_in_->emplace_back(pt.x, pt.y, pt.z, 0);
        }

        pcl::PointCloud<HEROPointXYZK>::Ptr out(new pcl::PointCloud<HEROPointXYZK>());
        out->reserve(temp_cloud_in_odin->points.size());
        for (const auto& pt : temp_cloud_in_odin->points) {
            out->emplace_back(pt.x, pt.y, pt.z, 1);
        }

        sor_->setInputCloud(out);
        sor_->setMeanK(50);
        sor_->setStddevMulThresh(1.0);
        
        pcl::PointCloud<HEROPointXYZK> filtered_out;
        sor_->filter(filtered_out);

        // cloud_in_->insert(cloud_in_->end(), filtered_out.begin(), filtered_out.end());
        *cloud_in_=*out+*cloud_in_;
        Eigen::Matrix4f tf_mat = Eigen::Matrix4f::Identity();
        try {
            tf_stamped = this->tf_buffer_->lookupTransform(
                "map", cloud_msg->header.frame_id, tf2::TimePointZero);
            tf_stamped.transform.translation.z += tf_z_offset_;  // z 偏移,防越界(参数化)
            const auto& tr = tf_stamped.transform.translation;
            const auto& rr = tf_stamped.transform.rotation;

            Eigen::Quaternionf q(
                static_cast<float>(rr.w), static_cast<float>(rr.x),
                static_cast<float>(rr.y), static_cast<float>(rr.z));
            q.normalize();

            tf_mat.setIdentity();
            tf_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
            tf_mat(0, 3) = static_cast<float>(tr.x);
            tf_mat(1, 3) = static_cast<float>(tr.y);
            tf_mat(2, 3) = static_cast<float>(tr.z);
            tf_stamped = this->tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);  // pb2025 机器人无 base_link 帧,用 base_footprint

        } catch (...) {
            std::cout << " -- [DOG] Cloud callback pcl::fromROSMsg error, "
                         "skip this frame."
                      << std::endl;
            return;
        }

        pcl::transformPointCloud(*cloud_in_, *cloud_in_, tf_mat);
        if (!cloud_in_->empty()) {
            auto msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*cloud_in_, msg);

            msg.header.frame_id = "map";
            msg.header.stamp = this->get_clock()->now();
            cost_pub_.ground_pc_pub->publish(std::move(msg));
        }
        rc_.update_lock.lock();
        rc_.pc = *cloud_in_;
        {
            rc_.pc_pose.first[0] = tf_stamped.transform.translation.x;
            rc_.pc_pose.first[1] = tf_stamped.transform.translation.y;
            rc_.pc_pose.first[2] = tf_stamped.transform.translation.z;
        }
        rc_.unfinished_frame_cnt++;
        map_empty_ = false;
        rc_.update_lock.unlock();
    }
#endif
    void updateCallback() {
        auto start_time = std::chrono::steady_clock::now();
        if (map_empty_) {
            static double last_print_t = this->get_clock()->now().seconds();
            double cur_t = this->get_clock()->now().seconds();
            if ((cur_t - last_print_t > 1.0)) {
                std::cout << " -- [DOG WARN] No point cloud input, check the "
                             "topic name."
                          << std::endl;
                last_print_t = cur_t;
            }
            return;
        }
        if (rc_.unfinished_frame_cnt == 0) {
            return;
        }

        if (rc_.unfinished_frame_cnt > 1) {
            std::cout
                << " -- [DOG WARN] Unfinished frame cnt > 1, the map may not "
                   "work in real-time"
                << std::endl;
        }
            static pcl::PointCloud<HEROPointXYZK>::Ptr temp_pc =
        std::make_shared<pcl::PointCloud<HEROPointXYZK>>();
        static Pose temp_pose;
        //从rc_中取出当前帧点云和位姿，进行地图更新
        rc_.update_lock.lock();
        temp_pc->clear();
        temp_pc->swap(rc_.pc);
        temp_pose = rc_.pc_pose;
        rc_.unfinished_frame_cnt = 0;
        rc_.update_lock.unlock();
        occ_map_->Update(temp_pc, temp_pose);
      
        pcl::PointCloud<pcl::PointXYZI> occ_pc;
        occ_map_->PubRes(occ_pc);
        
        if (!occ_pc.empty()) {
            auto msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(occ_pc, msg);

            msg.header.frame_id = "map";
            msg.header.stamp = this->get_clock()->now();
            cost_pub_.cost_pc_pub->publish(std::move(msg));
        }
       
#ifdef FIX_MAP
      static   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_fix=
            std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_in_fix->clear();
        cloud_in_fix->swap(occ_pc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr fix_pc =
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        occ_map_->GetStaticFixMap()->AdjustAndFixByRacying2d(
            temp_pose.first[0], temp_pose.first[1], cloud_in_fix, fix_pc);
        /************* */
        if (!fix_pc->empty()) {
            auto fix_msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*fix_pc, fix_msg);
            fix_msg.header.frame_id = "map";
            fix_msg.header.stamp = this->get_clock()->now();
            cost_pub_.fix_pc_pub->publish(fix_msg);
        } else {
            std::cout << " -- [DOG INFO] No fix point to publish." << std::endl;
        }
#endif
        /************* */
        auto end_time = std::chrono::steady_clock::now();
        double time_consuming =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                  start_time)
                .count();
         std::cout << " -- [DOG INFO] Map updated in " << time_consuming <<
         "ms." << std::endl;
    }

    DogMapNode() : Node("dog_map_node") {
        this->declare_parameter<std::string>("odin_topic", odin_topic);
        this->declare_parameter<std::string>("cloud_topic", cloud_topic);
        this->declare_parameter<std::string>("map_yaml_path", "");
        this->declare_parameter<double>("heighter_than_ground_threshold", 0.05);
        this->declare_parameter<double>("higher_than_car_threshold", 0.5);
        this->declare_parameter<int8_t>("LOG_OCC_HIT_mid360", 41);
        this->declare_parameter<int8_t>("LOG_OCC_HIT_odin1", 41);
        this->declare_parameter<int8_t>("LOG_OCC_FREE", -5);
        this->declare_parameter<uint8_t>("THR_OCC", 20);
        this->declare_parameter<uint8_t>("MAX_LOG_MID360", 100);
        this->declare_parameter<uint8_t>("MAX_LOG_ODIN1", 100);
        this->declare_parameter<uint8_t>("MIN_LOG", 10);
        int8_t LOG_OCC_HIT_MID360 = 40;   // 占据增量
        int8_t LOG_OCC_HIT_ODIN1 = 40;   // 占据增量
        int8_t LOG_OCC_FREE = -3;  // 空闲减量
        uint8_t THR_OCC = 30;      // 判定为障碍物的阈值
        uint8_t MAX_LOG_MID360 = 100;
        uint8_t MAX_LOG_ODIN1 = 100;    
        uint8_t MIN_LOG = 5;
        this->declare_parameter<double>("half_map_size", 10.0);
        this->declare_parameter<double>("resolution_z", 0.03);
        this->declare_parameter<int>("frame_save", 10);
        // map 系 z 偏移(原 HERO 车体 hack:tf 平移 z += 0.3 防越界;pb2025 融合默认 0.0)
        this->declare_parameter<double>("tf_z_offset", 0.0);
        this->get_parameter("LOG_OCC_HIT_mid360", LOG_OCC_HIT_MID360);
        this->get_parameter("LOG_OCC_HIT_odin1", LOG_OCC_HIT_ODIN1);
        this->get_parameter("LOG_OCC_FREE", LOG_OCC_FREE);
        this->get_parameter("THR_OCC", THR_OCC);
        this->get_parameter("MAX_LOG_MID360", MAX_LOG_MID360);
        this->get_parameter("MAX_LOG_ODIN1", MAX_LOG_ODIN1);
        this->get_parameter("MIN_LOG", MIN_LOG);
        std::cout <<" LOG_OCC_HIT "<<(int )LOG_OCC_HIT_MID360<<" "<<(int )LOG_OCC_HIT_ODIN1<<std::endl;
        std::cout <<" LOG_OCC_FREE "<<(int )LOG_OCC_FREE<<std::endl;
        std::cout <<" THR_OCC "<<(int )THR_OCC<<std::endl;
        std::cout <<" MAX_LOG_MID360 "<<(int )MAX_LOG_MID360<<std::endl;
        std::cout <<" MAX_LOG_ODIN1 "<<(int )MAX_LOG_ODIN1<<std::endl;
        std::cout <<" MIN_LOG "<<(int )MIN_LOG<<std::endl;
        this->get_parameter("frame_save", frame_save);
        this->get_parameter("odin_topic", odin_topic);
        this->get_parameter("resolution_z", resolution_z);
        this->get_parameter("cloud_topic", cloud_topic);
        this->get_parameter("map_yaml_path", map_yaml_path);
        this->get_parameter("heighter_than_ground_threshold",
                            heighter_than_ground_threshold);
        this->get_parameter("higher_than_car_threshold",
                            higher_than_car_threshold);
        this->get_parameter("half_map_size", half_map_size);
        this->get_parameter("tf_z_offset", tf_z_offset_);
       
        cloud_in_ = std::make_shared<pcl::PointCloud<HEROPointXYZK>>();
        const rclcpp::QoS qos(
            rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
          const rclcpp::QoS qos_sub(
            rclcpp::QoS(1).keep_last(1).durability_volatile());
        occ_map_ = new OccMap(heighter_than_ground_threshold,
                              higher_than_car_threshold, half_map_size,
                              resolution_z, frame_save, map_yaml_path);
        occ_map_->SetLOGParams(LOG_OCC_HIT_MID360, LOG_OCC_HIT_ODIN1, LOG_OCC_FREE, THR_OCC, MAX_LOG_MID360, MAX_LOG_ODIN1, MIN_LOG);
        cost_pub_.cost_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "rog_map/inf_occ", qos);
        cost_pub_.ground_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "rog_map/ground", qos);
#ifdef FIX_MAP
        cost_pub_.fix_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/fix",
                                                                  qos);
#endif
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        {
                
            rc_.cloud_me_cbk_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions so;
            
           
            so.callback_group = rc_.cloud_me_cbk_group;
            #ifndef ODIN_WIDTH_LIDAR 
            rc_.cloud_sub =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic, qos_sub,
                    std::bind(&DogMapNode::cloudCallback, this,
                              std::placeholders::_1),
                    so);
            #else
            rc_.cloud_sub_sync = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, cloud_topic, qos_sub.get_rmw_qos_profile());
            rc_.odin_sub_sync = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, odin_topic, qos_sub.get_rmw_qos_profile());
            rc_.sync = std::make_shared<message_filters::Synchronizer<ROSCallback::SyncPolicy>>(
                ROSCallback::SyncPolicy(10), *rc_.cloud_sub_sync, *rc_.odin_sub_sync);
            rc_.sync->registerCallback(std::bind(&DogMapNode::cloudCallback, this, std::placeholders::_1, std::placeholders::_2));
            sor_ = std::make_shared<pcl::StatisticalOutlierRemoval<HEROPointXYZK>>();
            #endif
            rc_.update_cbk_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            rc_.update_timer = this->create_wall_timer(
                std::chrono::milliseconds(1),  // 0.001秒，即1毫秒
                std::bind(&DogMapNode::updateCallback, this),
                rc_.update_cbk_group);
        }
    }
    ~DogMapNode() {
        if (occ_map_) delete occ_map_;
    }

   private:
    OccMap* occ_map_{nullptr};
    bool map_empty_{true};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string odin_topic = "odin1/slam", cloud_topic = "registered_scan";

    geometry_msgs::msg::TransformStamped tf_stamped;

    pcl::PointCloud<HEROPointXYZK>::Ptr cloud_in_;

   private:
#ifdef ODIN_WIDTH_LIDAR
    std::shared_ptr<pcl::StatisticalOutlierRemoval<HEROPointXYZK>> sor_;
#endif
    double half_map_size{10.0};
    double heighter_than_ground_threshold{0.05};
    double higher_than_car_threshold{0.5};
    double resolution_z{0.03};
    int frame_save{10};
    double tf_z_offset_{0.0};
    std::string map_yaml_path{""};
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DogMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}