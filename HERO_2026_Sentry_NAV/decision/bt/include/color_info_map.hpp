#pragma once
#include <cstdint>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace colorMap {
enum COLOR_ID { NONE = 0, RED = 1 };  // RED表示豁免区域
#define ERROR_CODE -100
/*
    yaml 读取指定索引内容哦
    */
template <typename T>
T yaml_get_value(const YAML::Node& node, const std::string& key) {
    try {
        return node[key].as<T>();
    } catch (YAML::Exception& e) {
        std::stringstream ss;
        ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
        throw YAML::Exception(e.mark, ss.str());
    }
}
class ColorMapModel {
   public:
    ColorMapModel(const std::string& color_map_file,
                  const std::string& yaml_file) {
        color_map_file_ = color_map_file;
        yaml_file_ = yaml_file;
        // 加载彩色区域地图图像
        ReadFileAndCreateMap();

        origin_ = std::vector<double>(2, 0.0);
        // 解析 YAML 文件获取地图信息
        YAML::Node node = YAML::LoadFile(yaml_file_);
        resolution_ = yaml_get_value<double>(node, "resolution");
        origin_ = yaml_get_value<std::vector<double>>(node, "origin");
    }

    bool World2ColorMap(double& wx, double& wy, unsigned& mx, unsigned& my) {
        if (resolution_ <= 0) {
            std::cerr << "Invalid resolution: " << resolution_ << "\n";
            return false;
        }
        mx = static_cast<unsigned>((wx - origin_[0]) / resolution_);
        my = static_cast<unsigned>((wy - origin_[1]) / resolution_);
        my = image_height_ - my -
             1;  // 图像坐标系y轴向下，世界坐标系y轴向上，需要转换
        if (mx >= image_width_ || my >= image_height_) {
            std::cerr << "World coordinates out of bounds: (" << wx << ", "
                      << wy << ") -> (" << mx << ", " << my << ")\n";
            return false;
        }
        return true;
    }
    void ReadFileAndCreateMap() {
        // 1. 彩色地图读取
        cv::Mat image = cv::imread(this->color_map_file_, 1);
        if (image.empty()) return;

        this->image_height_ = image.rows;
        this->image_width_ = image.cols;
        this->image_size_ = this->image_height_ * this->image_width_;

        // 释放旧内存（如果有的话），防止多次调用内存泄漏
        if (this->color_map_) delete[] this->color_map_;
        this->color_map_ = new uint8_t[this->image_size_];

        // 2. 转换为 HSV
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // 3. 设定 HSV 阈值（重点：限制 S 和 V 排除黑白灰）
        const uint8_t MIN_S = 60;  // 饱和度下限：过滤洗白了的淡颜色 / 白色
        const uint8_t MIN_V = 50;  // 亮度下限：过滤暗色 / 黑色

        // 使用 OpenCV 内置的多线程并行加速行处理
        cv::parallel_for_(
            cv::Range(0, hsv_image.rows), [&](const cv::Range& range) {
                for (int i = range.start; i < range.end; ++i) {
                    // 获取当前行的首地址
                    const cv::Vec3b* hsv_row_ptr = hsv_image.ptr<cv::Vec3b>(i);

                    for (int j = 0; j < hsv_image.cols; ++j) {
                        uint8_t h = hsv_row_ptr[j][0];
                        uint8_t s = hsv_row_ptr[j][1];
                        uint8_t v = hsv_row_ptr[j][2];

                        int curr_index = i * this->image_width_ + j;

                        // 核心修复：H 在红色区间，【并且】S 和 V 必须足够大
                        if (((h >= 0 && h <= 10) || (h >= 156 && h <= 180)) &&
                            (s >= MIN_S) && (v >= MIN_V)) {
                            this->color_map_[curr_index] = RED;
                        } else {
                            this->color_map_[curr_index] = NONE;
                        }
                    }
                }
            });

        // 4. 输出 color_map_ 到图片，调试用
        cv::Mat res_mat = cv::Mat::zeros(image.size(), CV_8UC3);  // 默认全黑
        for (int i = 0; i < res_mat.rows; ++i) {
            cv::Vec3b* res_row_ptr = res_mat.ptr<cv::Vec3b>(i);
            for (int j = 0; j < res_mat.cols; ++j) {
                int index = i * this->image_width_ + j;
                if (this->color_map_[index] == RED) {
                    res_row_ptr[j] = cv::Vec3b(0, 0, 255);  // BGR: 红色
                }
            }
        }

        cv::imwrite("/home/z/Downloads/read/s.png", res_mat);
    }
    /* 提取彩色区域地图指定位置信息*/
    inline int GetColorMapInfo(unsigned int mx, unsigned int my) {
        if (mx >= image_width_ || my >= image_height_) {
            std::cerr << "Pixel coordinates out of bounds: (" << mx << ", "
                      << my << ")\n";
            return ERROR_CODE;
        }
        uint8_t color = color_map_[my * image_width_ + mx];
        if (color == RED) {  // 红色区域
            return RED;
        } else {
            return NONE;  // 非红色区域
        }
    }

    void PrintMAPinfo() {
        std::cerr << "color_map_file_: " << color_map_file_ << std::endl;
        std::cerr << "yaml_file_: " << yaml_file_ << std::endl;
        std::cerr << "image_width_: " << image_width_ << std::endl;
        std::cerr << "resolution_: " << resolution_ << std::endl;
        std::cerr << "origin_: [" << origin_[0] << ", " << origin_[1] << "]"
                  << std::endl;
        std::cerr << "image_height_: " << image_height_ << std::endl;
    }

   private:
    /*
  初始化
  */

    // c参数
    std::string color_map_file_;
    std::string yaml_file_;
    size_t image_width_;
    double resolution_;
    std::vector<double> origin_;
    size_t image_height_;
    size_t image_size_;
    uint8_t* color_map_;
};

}  // namespace colorMap
