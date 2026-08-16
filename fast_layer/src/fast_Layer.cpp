#include "fast_layer/fast_Layer.hpp"

// #include <nav2_costmap_2d/footprint.hpp>
namespace fast_layer
{
FastLayer::FastLayer() {}
FastLayer::~FastLayer() {}
void FastLayer::onInitialize()
{
  std::string topics_string;

  declareParameter("pointcloud_topic", rclcpp::ParameterValue("pointcloud"));
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  default_value_ = 0;
  rolling_window_ = layered_costmap_->isRolling();
  was_reset_ = false;
  matchSize();
  node->get_parameter(name_ + ".pointcloud_topic", topics_string);
  rclcpp::QoS qos(10);
  qos.best_effort();
  pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topics_string, qos, std::bind(&FastLayer::pointCloudCallback, this, std::placeholders::_1));
}
void FastLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  //...
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  *min_x = std::min(*min_x, robot_x - getSizeInMetersX() / 2);
  *min_y = std::min(*min_y, robot_y - getSizeInMetersY() / 2);
  *max_x = std::max(*max_x, robot_x + getSizeInMetersX() / 2);
  *max_y = std::max(*max_y, robot_y + getSizeInMetersY() / 2);
  pcl::PointCloud<pcl::PointXYZ> cloud_temp;
  {
    std::lock_guard<std::mutex> lock(mutex_cloud);
    cloud_temp = cloud_;
  }
  if (cloud_temp.empty()) {
    return;
  }
  for (auto & point : cloud_temp.points) {
    double wx = point.x;
    double wy = point.y;
    // double p_cost = 0.0;
    unsigned int mx, my;
    // std::cout << "show map info" << std::endl;
    // std::cout << "getSizeInCellsX: " << getSizeInCellsX() << std::endl;
    // std::cout << "getSizeInCellsY: " << getSizeInCellsY() << std::endl;
    // std::cout << "getOriginX: " << getOriginX() << std::endl;
    // std::cout << "getOriginY: " << getOriginY() << std::endl;
    // std::cout << "getResolution: " << getResolution() << std::endl;
    if (worldToMap(wx, wy, mx, my)) {
      // std::cout << "point mxbbbbbbbbbb: " << mx << " my: " << my << std::endl;
      plists.emplace_back(mx, my);
      touch(wx, wy, min_x, min_y, max_x, max_y);
    }
  }
}

// void FastLayer::matchSize() {}

void FastLayer::reset() {}

void FastLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // std::lock_guard<std::mutex> lock(mutex_cloud);
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  for (const auto & point : plists) {
    // std::cout << "point mx: " << point.mx << " my: " << point.my << std::endl;
    // if (
    //   master_grid.getIndex(point.mx, point.my) <
    //   master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY())
    master_grid.setCost(point.mx, point.my, nav2_costmap_2d::LETHAL_OBSTACLE);
    // else {
    //   RCLCPP_WARN(logger_, "Point mx or my is out of bounds: mx=%u, my=%u", point.mx, point.my);
    // }
  }
  plists.clear();
}
void FastLayer::deactivate() {}
void FastLayer::activate() {}

void FastLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Process the incoming point cloud message
  std::string frame_id = msg->header.frame_id;
  global_frame_ = layered_costmap_->getGlobalFrameID();
  if (frame_id != global_frame_) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_->lookupTransform(global_frame_, frame_id, tf2::TimePointZero);
      Eigen::Matrix4f eigen_transform =
        tf2::transformToEigen(transform_stamped).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZ> cloud_temp;
      pcl::fromROSMsg(*msg, cloud_temp);
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
      pcl::transformPointCloud(cloud_temp, transformed_cloud, eigen_transform);
      std::lock_guard<std::mutex> lock(mutex_cloud);
      cloud_ = transformed_cloud;

    } catch (...) {
      RCLCPP_WARN(
        logger_, "Could not transform point cloud from frame %s to frame %s", frame_id.c_str(),
        global_frame_.c_str());
      return;
    }
  } else {
    std::lock_guard<std::mutex> lock(mutex_cloud);
    pcl::fromROSMsg(*msg, cloud_);
  }
}

}  // namespace fast_layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fast_layer::FastLayer, nav2_costmap_2d::Layer)