#include "dog_map/occ_map.hpp"

#include <oneapi/tbb/parallel_for.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <pcl/filters/radius_outlier_removal.h>
#include "dog_map/eigen_alias.hpp"

OccMap::OccMap(double heighter_than_ground_threshold,
               double higher_than_car_threshold, double half_map_size,
               double resolution_z, int frame_save,
               const std::string& pgm_yaml_path)
    : higher_than_ground_threshold_(heighter_than_ground_threshold),
      higher_than_car_threshold_(higher_than_car_threshold),
      half_map_size_(half_map_size),
      resolution_z(resolution_z),
      frame_save_(frame_save),
      pgm_yaml_path(pgm_yaml_path) {
    YAML::Node config = YAML::LoadFile(pgm_yaml_path);
    cloud_at_map_origin = yaml_get_value<std::vector<double>>(config, "origin");
    resolution = yaml_get_value<double>(config, "resolution");

    std::string pgm_path;
    int index = pgm_yaml_path.find_last_of("/");
    pgm_path = pgm_yaml_path.substr(0, index + 1) +
               yaml_get_value<std::string>(config, "image");
    cv::Mat map_img = cv::imread(pgm_path, cv::IMREAD_UNCHANGED);
    if (!map_img.empty()) {
        std::cout << " -- [DOG_MAP] Map loaded: size_x="
                  << map_img.cols * resolution
                  << ", size_y=" << map_img.rows * resolution << std::endl;
        InitMap(map_img);
    }
/*************fix map init */
#ifdef FIX_MAP
    static_fix_map_ = std::make_shared<static_fix_map::StaticFixMap>();
    static_fix_map_->InitPtr(std::make_shared<cv::Mat>(grid_map_), occ_map,
                             layer_z, fake_layer_z);
    static_fix_map_->SetMapInfo(cloud_at_map_origin[0], cloud_at_map_origin[1],
                                0.0, resolution);
#endif
    /*************888888888888888 */
    std::cout << " -- [DOG_MAP] OccMap initialized." << std::endl;
    std::cout << "[DOG_MAP] path " << pgm_path << std::endl;
    std::cout << " -- [DOG_MAP] half_map_size: " << half_map_size_
              << ", resolution_z: " << resolution_z << std::endl;
    std::cout<<" -- [DOG_MAP] OccMap resolution "<<resolution<<  std::endl;
    std::cout << " -- [DOG_MAP] higher_than_ground_threshold: "
              << higher_than_ground_threshold_
              << ", higher_than_car_threshold: " << higher_than_car_threshold_
              << std::endl;
    std::cout << " -- [DOG_MAP] frame_save: " << frame_save_ << std::endl;
}
void OccMap::InitMap(cv::Mat& grid_map) {
    grid_map_ = grid_map;
    layer_z = static_cast<int>(1.0 / resolution_z);  // 1m高度
    occ_map = new uint8_t[grid_map_.rows * grid_map_.cols * layer_z];
#ifndef DEBUG
    pub_map_ptr_ = new std::atomic<bool>[grid_map_.rows * grid_map_.cols];
#endif
    fake_layer_z = static_cast<int>(0.35 / resolution_z);  //平移避免越界
    std::fill_n(occ_map, grid_map_.rows * grid_map_.cols * layer_z, 0);
}
void OccMap::Clear() {
    std::fill_n(occ_map, grid_map_.rows * grid_map_.cols, 0);
}

void OccMap::TimeGoON() {  //长期未观测到的障碍物清除
    std::vector<int> to_erase;
    to_erase.reserve(time_list_.size() / 4);  // 预估

    for (auto& [index, last_frame] : time_list_) {
        if (update_frame - last_frame > frame_save_) {
            occ_map[index] =
                std::max(static_cast<int>(occ_map[index]) + LOG_OCC_FREE / 2,
                         static_cast<int>(MIN_LOG));
        }
        if (occ_map[index] < THR_OCC) {
            to_erase.push_back(index);
        }
    }

    // 批量删除
    for (int idx : to_erase) {
        time_list_.erase(idx);
    }
}
using MapIt = std::map<int, long long>::iterator;
void OccMap::PubRes(pcl::PointCloud<pcl::PointXYZI>& pub_pc) {
    //三维索引
#ifndef DEBUG
    std::fill_n(pub_map_ptr_, grid_map_.rows * grid_map_.cols, false);
#endif

    std::vector<std::pair<MapIt, bool>> iterators;
    iterators.reserve(time_list_.size());
    for (auto it = time_list_.begin(); it != time_list_.end(); ++it) {
        iterators.emplace_back(it, false);
    }    
 
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, iterators.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                auto& index = iterators[i].first->first;  // 获取当前索引
                int rem = index % (grid_map_.cols * grid_map_.rows);
                int idy = rem / grid_map_.cols;
                int idx = rem % grid_map_.cols;
#ifndef DEBUG
                int key = idx + idy * grid_map_.cols;
                if (pub_map_ptr_[key].exchange(true)) continue;

#endif
                // TODO 高层分析 符合要求,入集合
                int max_z_occ_index = -1;
                int min_z_occ_index = 0;
                for (int zi = layer_z - 1; zi >= 0; zi--) {
                    if (IsOccupied(idx, idy, zi)) {
                        max_z_occ_index = zi;
                        break;
                    }
                }
                for (int zi = 0; zi < layer_z; zi++) {
                    if (IsOccupied(idx, idy, zi)) {
                        min_z_occ_index = zi;
                        break;
                    }
                }
                int dzf_i= (max_z_occ_index - min_z_occ_index);
                double dzf = dzf_i * resolution_z;
                if (dzf < higher_than_ground_threshold_) continue;  //地面过滤
                int occ_count=0;
                for(int i=min_z_occ_index;i<max_z_occ_index;i++){
                        if(IsOccupied(idx,idy,i))
                             occ_count++;   
                        }
                double rate =1.0*occ_count/dzf_i;
                // if(rate<0.5 &&dzf<0.12) continue;
                if (dzf < 0.12 &&
                    (min_z_occ_index - fake_layer_z) * resolution_z > 0.2)
                    continue;  //过高过滤
                // 计算最大空洞大小
                int max_hole_size = 0;
                int curr_hole_size = 0;
                for (int zi = max_z_occ_index - 1; zi >= min_z_occ_index;
                     zi--) {
                    if (!IsOccupied(idx, idy, zi)) {
                        curr_hole_size++;
                    } else {
                        if (curr_hole_size > max_hole_size) {
                            max_hole_size = curr_hole_size;
                        }
                        curr_hole_size = 0;
                    }
                }
                if (max_hole_size * resolution_z > 0.15) continue;
                iterators[i].second = true;  // 标记为符合要求
            }
        });

    for (const auto& [it, is_valid] : iterators) {
        if (is_valid) {
            int index = it->first;
            int idz = index / (grid_map_.cols * grid_map_.rows);
            int rem = index % (grid_map_.cols * grid_map_.rows);
            int idy = rem / grid_map_.cols;
            int idx = rem % grid_map_.cols;
            pcl::PointXYZI pt;
            pt.x = idx * resolution + resolution / 2.0 + cloud_at_map_origin[0];
            pt.y = idy * resolution + resolution / 2.0 + cloud_at_map_origin[1];
            pt.z = idz * resolution_z + resolution_z / 2.0 +
                   cloud_at_map_origin[2];
            pt.intensity = pt.z;  // Set intensity to 1.0 (or any other value)
            pub_pc.points.push_back(pt);

#ifdef FIX_MAP
            UpdateStaticMap(idx, idy);
#endif
        }

    }
    
}
OccMap::~OccMap() { delete[] occ_map; }
//俯瞰地图是否占据,任一高度层占据即为占据
bool OccMap::IsOccupied(const Vec2i& idx) {
    for (int zi = 0; zi < layer_z; zi++) {
        int index = idx[0] + idx[1] * grid_map_.cols +
                    zi * grid_map_.cols * grid_map_.rows;
        if (occ_map[index] >= THR_OCC) return true;
    }
    return false;
}
//指定某一个格子是否占据
bool OccMap::IsOccupied(int index) {
    if (occ_map[index] >= THR_OCC) return true;
    return false;
}
bool OccMap::IsOccupied(int ix, int iy, int iz) {
    int index = ix + iy * grid_map_.cols + iz * grid_map_.cols * grid_map_.rows;
    if (occ_map[index] >= THR_OCC) return true;
    return false;
}
bool OccMap::IsOccupied(const Vec3i& idx) {
    int index = idx[0] + idx[1] * grid_map_.cols +
                idx[2] * grid_map_.cols * grid_map_.rows;
    if (occ_map[index] >= THR_OCC) return true;
    return false;
}

bool OccMap::IsStaticMapPoint(const Vec2i& idx) {
    if (grid_map_.at<unsigned char>(idx[1], idx[0]) == 255) return true;
    return false;
}
bool OccMap::IsStaticMapPoint(int index) {
    int idx = index % grid_map_.cols;
    int idy = index / grid_map_.cols;

    if (grid_map_.at<unsigned char>(idy, idx) == 255) return true;

    return false;
}
void OccMap::racyHandle(const Vec3i& start_idx,
                        const std::vector<Vec3i>& obs_list) {
    // TODO
    // Bresenham算法实现3D网格的射线处理,发现符合障碍物标准,加入time_list_
}

void OccMap::PassFilter(pcl::PointCloud<HEROPointXYZK>::Ptr cloud_in,
                        Pose pose) {
    //高度滤波
    pcl::PassThrough<HEROPointXYZK> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.03, higher_than_car_threshold_ + 0.3);
    pass.filter(*cloud_in);
    
    double min_x = pose.first[0] - half_map_size_;
    min_x = std::max(min_x, cloud_at_map_origin[0] + 0.2);
    double max_x = pose.first[0] + half_map_size_;
    max_x = std::min(
        max_x, resolution * grid_map_.cols + cloud_at_map_origin[0] - 0.2);
    double min_y = pose.first[1] - half_map_size_;
    min_y = std::max(min_y, cloud_at_map_origin[1] + 0.2);
    double max_y = pose.first[1] + half_map_size_;
    max_y = std::min(
        max_y, resolution * grid_map_.rows + cloud_at_map_origin[1] - 0.2);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*cloud_in);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_in);
      crop_box_filter_.setMin(Eigen::Vector4f(crop_min_x+pose.first[0], crop_min_y+pose.first[1],
                                                crop_min_z,
                                                1.0f));  // 设置最小点坐标
        crop_box_filter_.setMax(Eigen::Vector4f(crop_max_x+pose.first[0], crop_max_y+pose.first[1],
                                                crop_max_z,
                                                1.0f));  // 设置最大点坐标
        crop_box_filter_.setNegative(
            true);  // 设置为true表示保留框外的点，去除框内的点
      crop_box_filter_.setInputCloud(cloud_in);
        crop_box_filter_.filter(*cloud_in);
    // 离群点滤波
    // if(cloud_in->size()==0) return ;
    //  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    // ror.setInputCloud(cloud_in);
    // ror.setRadiusSearch(0.1); // 设置搜索半径 (如 0.15 米)
    // ror.setMinNeighborsInRadius(4); // 设定在该半径内至少所需的邻居点数，小于此数则视为离群点过滤掉
    // ror.filter(*cloud_in);
    
}
void OccMap::Update(pcl::PointCloud<HEROPointXYZK>::Ptr cloud, Pose pose) {
    //更新局部地图

    PassFilter(cloud, pose);
       
    // TODO: 进行地图更新
    //射线起点格子
    int px =
        static_cast<int>((pose.first[0] - cloud_at_map_origin[0]) / resolution);
    int py =
        static_cast<int>((pose.first[1] - cloud_at_map_origin[1]) / resolution);
    int pz = static_cast<int>(pose.first[2] / resolution_z);
    Vec3i start_idx = Vec3i{px, py, pz};

    Vec3f res(resolution, resolution, resolution_z);

    // 2. 转换为物理空间坐标 (以中心或原点对齐)
    Vec3f start_f = start_idx.cast<double>().array() * res.array();
#ifndef TBB_DO

    for (auto& point : cloud->points) {
        int xi =
            static_cast<int>((point.x - cloud_at_map_origin[0]) / resolution);
        int yi =
            static_cast<int>((point.y - cloud_at_map_origin[1]) / resolution);
        int zi = static_cast<int>(point.z / resolution_z);
        if (zi < 0) continue;
        Vec3i obs_idx = Vec3i{xi, yi, zi};
        racyHandle(start_idx, obs_idx, start_f, res);
    }

#else
    // TODO: 并行化free空间更新

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, cloud->points.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                int xi = static_cast<int>(
                    (cloud->points[i].x - cloud_at_map_origin[0]) / resolution);
                int yi = static_cast<int>(
                    (cloud->points[i].y - cloud_at_map_origin[1]) / resolution);

                int zi = static_cast<int>(cloud->points[i].z / resolution_z);
                if (zi < 0) continue;
                Vec3i obs_idx = Vec3i{xi, yi, zi};
                racyHandle(start_idx, obs_idx, start_f, res);
            }
        });
    //障碍物添加

#endif

    for (auto& point : cloud->points) {
        int xi =
            static_cast<int>((point.x - cloud_at_map_origin[0]) / resolution);
        int yi =
            static_cast<int>((point.y - cloud_at_map_origin[1]) / resolution);
#ifdef SKIP_STATIC
        if (grid_map_.at<unsigned char>(grid_map_.rows - 1 - yi, xi) < 240)
            continue;  //静态障碍物不处理,偷运算
#endif
        int zi = static_cast<int>(point.z / resolution_z);
        if (zi > layer_z - 1) {
            //   // std::cout<<"zi "<<zi<<std::endl;
            continue;
        }  //地面滤波
        int index =
            xi + yi * grid_map_.cols + zi * grid_map_.cols * grid_map_.rows;
        //更新占据概率
        occ_map[index] =
            std::min(static_cast<int>(occ_map[index]) + LOG_OCC_HIT[point.device_id],
                     static_cast<int>(MAX_LOG[point.device_id]));
        //加入观测列表
        if (occ_map[index] >= THR_OCC) {
            time_list_[index] = update_frame;
        }
    }
       
    // 更新未观测到的累计帧数
    TimeGoON();
     
    update_frame++;

}
void OccMap::UpdateStaticMap(int idx, int idy) {
    int img_idy = grid_map_.rows - 1 - idy;
    if (idx < 0 || idx >= grid_map_.cols || idy < 0 || idy >= grid_map_.rows)
        return;
    if (grid_map_.at<unsigned char>(img_idy, idx) > 252) {
        grid_map_.at<unsigned char>(img_idy, idx) =
            0;  // 将 pgm 上的占据点改为 0（占据）
    }
}
