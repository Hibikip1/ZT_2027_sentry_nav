# hero_lidar_scan

双Livox Mid-360激光雷达去畸变融合节点

## 功能特性

- 接收高频外部里程计（100-500Hz）进行运动学去畸变
- 支持两台硬同步Livox Mid-360雷达数据融合
- 使用SLERP四元数插值和线性位置插值
- OpenMP多线程加速点云处理
- 输出标准PointCloud2格式

## 编译

```bash
cd /home/liu/project/hero2026_-sentry
colcon build --packages-select hero_lidar_scan
source install/setup.bash
```

## 运行

```bash
ros2 launch hero_lidar_scan hero_lidar_scan.launch.py
```

## 配置参数

编辑 `config/params.yaml` 设置：
- 雷达话题名称
- 里程计话题名称
- 雷达外参（平移和旋转）

## 核心算法

去畸变公式：
```
P_undistorted = Pose_end^-1 × Pose_i × T_ext × P_raw
```

其中：
- `Pose_end`: 帧结束时刻车体位姿
- `Pose_i`: 点扫描时刻车体位姿
- `T_ext`: 雷达到base_link外参
- `P_raw`: 原始点坐标
