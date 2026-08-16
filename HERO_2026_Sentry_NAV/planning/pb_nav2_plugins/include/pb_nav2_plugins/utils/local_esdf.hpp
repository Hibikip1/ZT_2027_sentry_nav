// Copyright 2026 Jinbo Liu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_NAV2_PLUGINS__UTILS__LOCAL_ESDF_HPP_
#define PB_NAV2_PLUGINS__UTILS__LOCAL_ESDF_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_msgs/msg/costmap.hpp"
#include "opencv2/imgproc.hpp"

namespace pb_nav2_plugins
{

/**
 * @brief Lightweight local ESDF (Euclidean Signed Distance Field) utility.
 *
 * Computes a signed distance field directly from a nav2_msgs::msg::Costmap
 * message using OpenCV's distanceTransform. Designed for the BackUpFreeSpace
 * behavior to determine escape direction via gradient queries.
 *
 * Design principles:
 *   - Header-only, no external package dependencies beyond Nav2 and OpenCV.
 *   - Bilinear interpolation for distance queries.
 *   - Central-difference gradient computation.
 */
class LocalESDF
{
public:
  LocalESDF() = default;

  /**
   * @brief Build the signed ESDF from a GetCostmap service response.
   *
   * Cells with cost >= 253 (INSCRIBED_INFLATED_OBSTACLE) are treated as
   * obstacles. The resulting field stores positive distances in free space
   * and negative distances inside obstacles.
   *
   * @param costmap  The costmap message from nav2_msgs::srv::GetCostmap.
   * @param max_distance  Maximum absolute distance (metres) stored in the
   *                      field. Values beyond this are clamped.
   * @return true on success, false if the costmap is empty.
   */
  bool updateFromCostmapMsg(const nav2_msgs::msg::Costmap & costmap, double max_distance = 5.0)
  {
    size_x_ = costmap.metadata.size_x;
    size_y_ = costmap.metadata.size_y;
    resolution_ = costmap.metadata.resolution;
    origin_x_ = costmap.metadata.origin.position.x;
    origin_y_ = costmap.metadata.origin.position.y;

    if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0) {
      valid_ = false;
      return false;
    }

    // --- Build binary obstacle image ---
    // obstacle: 0 (foreground), free: 255 (background)
    cv::Mat obstacle_img(size_y_, size_x_, CV_8UC1);
    for (unsigned int y = 0; y < size_y_; ++y) {
      for (unsigned int x = 0; x < size_x_; ++x) {
        unsigned int idx = y * size_x_ + x;
        obstacle_img.at<uchar>(y, x) = (costmap.data[idx] >= 253) ? 0 : 255;
      }
    }

    // --- Outside distance (free-space positive) ---
    cv::Mat dist_out;
    cv::distanceTransform(obstacle_img, dist_out, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // --- Inside distance (obstacle interior negative) ---
    cv::Mat inv_img;
    cv::bitwise_not(obstacle_img, inv_img);
    cv::Mat dist_in;
    cv::distanceTransform(inv_img, dist_in, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // --- Merge into signed distance field ---
    esdf_.resize(static_cast<size_t>(size_x_) * size_y_);
    const float max_d = static_cast<float>(max_distance);
    for (unsigned int y = 0; y < size_y_; ++y) {
      for (unsigned int x = 0; x < size_x_; ++x) {
        float d_out = dist_out.at<float>(y, x);
        float d_in = dist_in.at<float>(y, x);
        float d = (d_out > 0.0f) ? d_out * static_cast<float>(resolution_)
                                 : -d_in * static_cast<float>(resolution_);
        esdf_[y * size_x_ + x] = std::clamp(d, -max_d, max_d);
      }
    }

    valid_ = true;
    return true;
  }

  /**
   * @brief Bilinear-interpolated signed distance at world coordinate (x, y).
   * @return Distance in metres; positive = free, negative = inside obstacle.
   */
  double getDistance(double x, double y) const
  {
    if (!valid_) {
      return 0.0;
    }
    double lx = (x - origin_x_) / resolution_ - 0.5;
    double ly = (y - origin_y_) / resolution_ - 0.5;
    return bilinear(lx, ly);
  }

  /**
   * @brief Central-difference gradient of the signed distance field.
   *
   * The gradient points towards increasing distance, i.e. away from
   * the nearest obstacle.
   *
   * @param[out] gx  Gradient component along X (metres^-1... actually
   *                  dimensionless since distance is in metres and coords
   *                  are in metres).
   * @param[out] gy  Gradient component along Y.
   */
  void getGradient(double x, double y, double & gx, double & gy) const
  {
    if (!valid_) {
      gx = 0.0;
      gy = 0.0;
      return;
    }
    const double h = resolution_;
    double dp_x = getDistance(x + h, y);
    double dm_x = getDistance(x - h, y);
    double dp_y = getDistance(x, y + h);
    double dm_y = getDistance(x, y - h);
    gx = (dp_x - dm_x) / (2.0 * h);
    gy = (dp_y - dm_y) / (2.0 * h);
  }

  /**
   * @brief Check whether (x, y) falls inside the ESDF grid with a 1-cell margin.
   */
  bool isInside(double x, double y) const
  {
    if (!valid_) {
      return false;
    }
    double lx = (x - origin_x_) / resolution_;
    double ly = (y - origin_y_) / resolution_;
    return lx >= 1.0 && lx < (size_x_ - 1) && ly >= 1.0 && ly < (size_y_ - 1);
  }

  bool isValid() const { return valid_; }
  double getResolution() const { return resolution_; }

private:
  /**
   * @brief Bilinear interpolation on the ESDF grid.
   *
   * @param lx  Continuous grid coordinate (column), 0-based.
   * @param ly  Continuous grid coordinate (row), 0-based.
   */
  double bilinear(double lx, double ly) const
  {
    int ix = static_cast<int>(std::floor(lx));
    int iy = static_cast<int>(std::floor(ly));

    // Clamp to valid range
    ix = std::clamp(ix, 0, static_cast<int>(size_x_) - 2);
    iy = std::clamp(iy, 0, static_cast<int>(size_y_) - 2);

    double fx = lx - ix;
    double fy = ly - iy;
    fx = std::clamp(fx, 0.0, 1.0);
    fy = std::clamp(fy, 0.0, 1.0);

    double v00 = esdf_[iy * size_x_ + ix];
    double v10 = esdf_[iy * size_x_ + ix + 1];
    double v01 = esdf_[(iy + 1) * size_x_ + ix];
    double v11 = esdf_[(iy + 1) * size_x_ + ix + 1];

    return v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy) + v01 * (1 - fx) * fy + v11 * fx * fy;
  }

  bool valid_{false};
  std::vector<float> esdf_;
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
};

}  // namespace pb_nav2_plugins

#endif  // PB_NAV2_PLUGINS__UTILS__LOCAL_ESDF_HPP_
