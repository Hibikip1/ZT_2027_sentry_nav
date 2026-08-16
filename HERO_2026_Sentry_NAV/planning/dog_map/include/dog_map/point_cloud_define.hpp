#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

// 1. 定义数据结构体
struct alignas(16) HEROPointXYZK {
    PCL_ADD_POINT4D; // 这个宏实际上已经自带了匿名 union 和相关的 alignas 配置
    uint32_t device_id;
    
    // 构造函数
    inline HEROPointXYZK(const HEROPointXYZK &p) {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        device_id = p.device_id;
    }

    inline HEROPointXYZK(uint32_t _device_id = 0) 
        : HEROPointXYZK(0.f, 0.f, 0.f, _device_id) {}

    inline HEROPointXYZK(float _x, float _y, float _z, uint32_t _device_id = 0) {
        x = _x; y = _y; z = _z;
        data[3] = 1.0f;
        device_id = _device_id;
    }
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 2. 注册点云结构体 (必须针对实际用于 PCL 模板实例化的具体类型名进行注册)
POINT_CLOUD_REGISTER_POINT_STRUCT(HEROPointXYZK,
    (float, x, x) 
    (float, y, y)
    (float, z, z)
    (uint32_t, device_id, device_id)
)