
#include "dog_map/point_cloud_define.hpp"
// 必须在包含 impl 之后进行显式实例化
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/passthrough.hpp>

// 显式实例化模板类，这能解决 undefined reference to `pcl::PCLBase<...>`
template class pcl::PCLBase<HEROPointXYZK>;
template class pcl::Filter<HEROPointXYZK>;
template class pcl::PassThrough<HEROPointXYZK>;