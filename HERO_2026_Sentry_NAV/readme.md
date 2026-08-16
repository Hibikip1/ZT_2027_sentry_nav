# HERO 2026 赛季哨兵导航算法开源

**作者：** 哈尔滨工业大学（威海）HERO 竞技机器人实验室 25,26赛季导航组

- 刘谨搏（[GitHub](https://github.com/LiuJinbo1027)）
- 赵家康（[GitHub](https://github.com/zh666-ovo)）
- 冯永康（[GitHub](https://github.com/Rokiii1012)）

---

## 效果展示

- [和我的 MINCO + MPC 说去吧](https://www.bilibili.com/video/BV1u7XZBrEPC/)
- [留形科技 & HERO 产品测试（北部分区赛前）](https://www.bilibili.com/video/BV16eVJ6ZEDm/)
- [北部赛区 第七十七场：哈工大（威海）HERO vs 西安交大 笃行](https://www.bilibili.com/video/BV1vE5F6wELq/)

---

## 项目内容

### 1. 决策层 `decision`

能量机关状态机节点、追击节点、优先级打断逻辑。开源 2025、2026 赛季赛场真实行为树策略。

### 2. 感知层前端 `perception`

- **双激光雷达融合**（`lidar_merge`）：融合双雷达点云，扩大感知范围。
- **全稠密点云全景感知**（`hero_lidar_scan`）：基于 Odin1 去除点云运动畸变，实现 360° 稠密点云感知。
- **双端融合态势感知**（`nav_cv_bridge`）：融合雷达点云与视觉自瞄数据，提升感知鲁棒性。

### 3. 双机通信方案 `zenoh-bridge-ros2dds`

通过网线直连导航与自瞄两台计算机，基于 PTP 时间戳硬同步，实现双机之间直接的 ROS 2 通信。

### 4. 感知层后端 `dog_map` / `fast_layer`

参考 ROG-Map 实现点云后端分析与感知，并向 `nav2-costmap` 投影生成代价地图。

### 5. MPC 控制器

基于 ACADOS 工具库封装 `nav2-controller` 节点，实现稳定高效的 MPC 轨迹全状态追踪。

### 6. MINCO 优化器

基于最小化控制理论（MINCO）的轨迹优化器，包含：

- ESDF 避障
- 二次轨迹优化
- 基于最近投影点的重规划策略

### 7. 等待与回退策略 `nav2_plugins`

提供误卡入障碍物时的回退策略，以及道路被堵时的等待策略，使导航行为树更加稳健。

### 8. 机器人中心-激光雷达外参标定工具

`lidar_extrinsic_calibration.py`：基于最小二乘法进行圆形轨迹拟合，输出机器人中心相对于激光雷达里程计的 x、y 偏移量。

---

## 未来展望

1. **狭窄隧道穿越**：本套导航算法经测试可在 10 cm 裕度下完成隧道穿越。遗憾本赛季未能实现过洞哨兵的完整制作，未来需加入更优雅的指定区域正向朝向约束逻辑。
2. **模糊地图引导的自主探索**：鉴于各战队普遍反映仿真与实际场地差异较大，且官方收紧建图权限的趋势，基于模糊先验地图的自主探索已成为必然方向。在 Navigation 2 架构下：
   - **global_map** 应由仿真点云导出，为 planner 提供先验引导；
   - **local_map** 完全由点云实时分析投影获得，为 smoother 和 controller 提供局部精确地图。
   - 可参考香港大学 SUPER 团队的 ROG-Map + 局部 MINCO 优化的思路。

---

## 参考与致谢

1. 陈立憨 — 深圳北理莫斯科大学（[GitHub](https://github.com/LihanChen2004)）
2. 张昊鹏 — 中国科学技术大学（[GitHub](https://github.com/ZhangHaopeng-Dino)）
3. 浙江大学 — GCOPTER、DDR-Opt
4. 喻衡 — 北京理工大学（[GitHub](https://github.com/Walker152)）
5. 哈尔滨工业大学（威海）HERO 战队 2025、2026 赛季全体队员

---

## 写在最后

> 为 RoboMaster 的技术进步留下些什么，才不枉疯狂过一场。
> 
> 以此，致我的两年 RM 青春。
