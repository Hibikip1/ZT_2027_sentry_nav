# 激光雷达外参标定工具

## 功能说明

本工具用于对安装在**旋转云台**上的激光雷达进行外参标定，计算：

1. **旋转半径 (Radius)**: 雷达中心到云台旋转轴的水平距离
2. **安装倾角 (Pitch Bias)**: 雷达相对于水平面的俯仰角

## 使用场景

- 机器人底盘保持**绝对静止**
- 云台带动激光雷达进行 **360° 旋转**
- 录制 FAST-LIVO 输出的里程计数据

## 安装依赖

```bash
pip install rosbags numpy scipy matplotlib
```

## 数据采集

1. 将机器人放置在**平坦地面**上
2. 启动 FAST-LIVO，等待初始化完成
3. 开始录制 bag：
   ```bash
   ros2 bag record /aft_mapped_to_init -o calibration_data
   ```
4. 控制云台以**均匀速度**旋转 1-3 圈
5. 停止录制

## 运行标定

```bash
# 基本用法
python3 lidar_extrinsic_calibration.py /path/to/calibration_data

# 指定话题名称
python3 lidar_extrinsic_calibration.py /path/to/bag -t /custom/odom_topic

# 保存图片而不显示
python3 lidar_extrinsic_calibration.py /path/to/bag -o result.png

# 只输出数值，不显示图形
python3 lidar_extrinsic_calibration.py /path/to/bag --no-plot
```

## 输出示例

```
============================================================
标定结果总结
============================================================

  【旋转半径 (Radius)】
     R = 0.199600 m = 199.600 mm
     (用于填入 t_lidar_joint 的 x 分量)

  【安装倾角 (Pitch Bias)】
     Pitch = 0.610865 rad = 35.0000 deg
     (用于填入 angle_y 参数)

  【拟合质量】
     RMSE = 2.345 mm
     ✓ 拟合质量优秀 (RMSE < 5mm)

============================================================
```

## 应用标定结果

将标定结果填入 `LIVMapper.cpp` 中的 `publish_odometry` 函数：

```cpp
// base_link → joint_link: 绕 Y 轴旋转 (Pitch)
double angle_y = 0.610865;  // 标定得到的 Pitch 角 (弧度)

// joint_link → lidar_link: X 方向平移 (半径)
Eigen::Vector3d t_lidar_joint(0.1996, 0.0, 0.0);  // 标定得到的半径 (米)
```

## 验证方法

1. 将标定参数填入代码并重新编译
2. 再次让底盘静止、云台旋转
3. 观察 `/aft_mapped_to_init` 的 XY 轨迹
4. 如果标定准确，原来的**大圆圈**应该收敛为一个**静止点**

## 可视化说明

脚本会生成两张子图：

1. **左图 (XY 平面)**：显示原始轨迹散点和拟合圆，颜色表示时间进度
2. **右图 (姿态变化)**：显示 Roll/Pitch 随时间的变化，红线表示平均 Pitch

## 注意事项

- 确保录制数据时 FAST-LIVO 已完成初始化（轨迹不再剧烈飘移）
- 云台旋转速度应均匀，避免突然加减速
- 如果 RMSE 较大（>10mm），检查是否有外部干扰或底盘移动
