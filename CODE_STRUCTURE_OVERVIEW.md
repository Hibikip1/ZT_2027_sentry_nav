# 代码结构总览:两套 RoboMaster 哨兵导航

> 本文档为后续「两个项目部分融合」任务的前置阅读笔记,记录两个开源导航项目(均为 ROS2 Humble / NAV2 框架)的结构、职责与差异。内容基于对启动文件、`nav2_params.yaml`、`package.xml`、行为树 XML 及关键源码的逐包精读与交叉验证(两个独立的深度阅读代理 + 人工复核)。

本仓库实际包含两套相互独立、又有渊源关系的哨兵导航代码:

| 位置 | 项目 | 团队 | 定位 |
| --- | --- | --- | --- |
| 根目录(本仓库) | **pb2025_sentry_nav** | 深圳北理莫斯科大学 北极熊战队 (SMBU) | 2025 赛季哨兵导航,仿真 + 实车,教学向,工程化完善 |
| `HERO_2026_Sentry_NAV/` | **HERO 2026 赛季哨兵导航** | 哈工大(威海)HERO 竞技机器人实验室 | 2025/2026 赛季,实车比赛,算法激进(MPC + MINCO + 行为树决策) |

**关键关系**:HERO 项目在 `hero2025_nav_bringup` 的 `bringup_launch.py` 与 `navigation_launch.py` 中大量沿用 pb2025 的结构(`terrain_analysis`/`terrain_analysis_ext` 节点声明、`BackUpFreeSpace` 行为、`<robot_namespace>` 命名空间替换逻辑、`RewrittenYaml` 参数注入、`cmd_vel` remap 链路),`bringup_launch.py` 版权头即 pb2025 作者 Lihan Chen(带中文注释翻译)。可确认 **HERO 是在 pb2025 基础上二次开发**(nav2 bringup 层高度同源),融合具备天然基础。

---

## 一、项目一:pb2025_sentry_nav(根目录)

### 1.1 概览

- 框架:NAV2 (ros-navigation/navigation2),参考 `autonomous_exploration_development_environment` 设计(terrain_analysis 源码版权即 Hongbiao Zhu)。
- 环境:Ubuntu 22.04 + ROS2 Humble,支持 Gazebo 仿真与实车两种模式(`config/simulation` 与 `config/reality` 两套参数)。
- 传感器:Livox mid360(倾斜侧放),点云 + IMU 做 LiDAR 惯性里程计;仿真中为 velodyne 样式点云(需 `ign_sim_pointcloud_tool` 补 `time` field)。
- 命名空间:全面引入 `namespace`(仿真默认 `red_standard_robot1`,实车默认空 `""`),为多机器人扩展做准备。
- 工程化:README 中英文、Wiki、CI(`build_and_test.yml` + `docker.yml`)、pre-commit 完善;meta 包版本 1.3.2。

### 1.2 包清单(根目录)

| 包 | 性质 | 职责 |
| --- | --- | --- |
| `pb2025_sentry_nav` | 本地 | 元包(仅声明依赖,无代码) |
| `pb2025_nav_bringup` | 本地 | 启动文件、`nav2_params.yaml`(仿真/实车)、行为树 XML、地图/PCD/RViz |
| `loam_interface` | 本地 | 里程计接口:把 point_lio 输出从 `camera_init` 系重锚定到 `odom` 系 |
| `sensor_scan_generation` | 本地 | 核心变换节点:广播 `odom→base_footprint`、发布 `odometry`(odom→gimbal_yaw)、重发点云到雷达系 |
| `fake_vel_transform` | 本地 | 虚拟速度参考系,应对云台扫描自旋 |
| `terrain_analysis` | 本地 | 车体 4m 内地形分析,障碍物离地高度写入点云 intensity(Hongbiao Zhu 源码) |
| `terrain_analysis_ext` | 本地 | 车体 4m 外地形分析(全局代价地图用) |
| `ign_sim_pointcloud_tool` | 本地 | 仿真器点云补 `time`/`ring` field、转 velodyne 格式 |
| `pointcloud_to_laserscan` | 子模块 | 将 `terrain_map_ext` 转 LaserScan(仅 SLAM 建图模式) |
| `point_lio` | 子模块 | LiDAR 惯性里程计(分支 RM2025_SMBU_auto_sentry) |
| `small_gicp_relocalization` | 子模块 | small_gicp 全局重定位(先验 PCD + GICP 配准) |
| `pb_omni_pid_pursuit_controller` | 子模块 | 全向 PID + Pure Pursuit 路径跟踪控制器 |
| `pb_nav2_plugins` | 子模块 | nav2 扩展插件:`IntensityVoxelLayer` 代价层 + `BackUpFreeSpace` 行为 |
| `pb_teleop_twist_joy` | 子模块 | PS4 手柄控制(底盘 + 云台) |
| `livox_ros_driver2` | 子模块 | Livox 雷达驱动 |

> 子模块清单见 `.gitmodules`(7 个)。注意 `point_lio` 在 `git status` 中显示为未跟踪(`? point_lio`),子模块可能未正确初始化。

### 1.3 TF / 坐标变换流水线(核心设计)

坐标变换是 pb2025 重点优化的部分(README 原文:"大幅优化了坐标变换逻辑,考虑雷达原点 `lidar_odom` 与底盘原点 `odom` 之间的隐式变换")。完整数据流(源码验证):

```
point_lio (pointlio_mapping)
  sub: livox/lidar(实车) 或 velodyne_points(仿真) + livox/imu
  pub: aft_mapped_to_init (Odometry, camera_init→body 系)
       cloud_registered  (PointCloud2)

loam_interface                      # 一次 TF 查找(odom→front_mid360 的"隐式变换")
  sub: aft_mapped_to_init, cloud_registered
  pub: lidar_odometry (Odometry, odom→front_mid360 系)      # 把位姿重锚定到 odom
       registered_scan (PointCloud2, odom 系)               # 把点云重锚定到 odom

sensor_scan_generation              # 核心变换节点,message_filters 近似时间同步(队列100)
  sub: lidar_odometry, registered_scan
  pub: sensor_scan (PointCloud2, front_mid360 系)
       odometry    (Odometry, odom→gimbal_yaw 系, twist 由数值微分得到)
  TF : odom → base_footprint

small_gicp_relocalization           # 全局重定位
  sub: registered_scan, initialpose
  TF : map → odom (GICP, 先验 PCD 上配准, 参数 num_threads=4, leaf 0.1/0.05)

fake_vel_transform                  # 50Hz 定时发布 TF + 速度变换
  sub: odometry, local_plan(时间戳对齐用), cmd_spin(Float32), cmd_vel_nav2_result
  pub: cmd_vel (gimbal_yaw 系, 叠加自旋)
  TF : gimbal_yaw → gimbal_yaw_fake (yaw = −当前底盘朝向)
```

**帧树**:

```
map ──(small_gicp / SLAM 模式静态tf)──> odom ──(sensor_scan_generation TF)──> base_footprint
                                             └──(odometry 话题)──> gimbal_yaw ──(fake_vel_transform)──> gimbal_yaw_fake
                                                                         └── front_mid360 (雷达系)
```

关键设计点(源码验证):
1. **两套里程计帧被桥接**:point_lio 输出 `aft_mapped_to_init`(camera_init 系),`loam_interface` 用 TF 查找 `base_footprint→front_mid360` 得到隐式变换 `tf_odom_to_lidar_odom_`,重锚定到 `odom`;`sensor_scan_generation` 再广播 `odom→base_footprint` 并发布 nav2 消费的 `odometry`(`odom→gimbal_yaw`,twist 用相邻两帧位姿数值微分,`sensor_scan_generation.cpp:131-152`)。
2. **`intensity` 字段被复用作"离地高度"**:`terrain_map`/`terrain_map_ext` 把可通行性高程编码进点云 `intensity`,`IntensityVoxelLayer` 过滤 `[0.1, 2.0] m` 内的点标记为障碍。
3. **`gimbal_yaw_fake` 是 nav2 的参考系**:云台持续自旋会破坏局部规划器(假定机头朝向 = 路径朝向),`fake_vel_transform` 建立 yaw 恒向前的虚拟系(`publishTransform` 50Hz,旋转 = −当前 yaw),把结果命令 `transformVelocity` 转回 `gimbal_yaw` 并叠加 `cmd_spin`(`angular.z = cmd.angular.z + spin_speed_`)。
4. **不控制航向**:`enable_rotation: false`、`yaw_goal_tolerance: 6.28`,控制器纯全向平移,航向由云台/自旋系统单独处理。
5. **速度平滑链路**:controller(20Hz)输出 `cmd_vel_controller` → velocity_smoother → `cmd_vel_nav2_result` → fake_vel_transform(叠加自旋)→ 底盘 `cmd_vel`。

### 1.4 规划 / 控制配置(仿真 nav2_params.yaml 验证)

| 组件 | 插件 | 说明 |
| --- | --- | --- |
| Global Planner | `nav2_theta_star_planner/ThetaStarPlanner` | Theta* 全局规划,how_many_corners=8, w_traversal_cost=10 |
| Smoother | `nav2_smoother::SimpleSmoother` | 简单平滑(max_its=1000) |
| Controller | `pb_omni_pid_pursuit_controller::OmniPidPursuitController` | 20Hz,全向 PID + 纯追踪,平移/旋转双 PID,曲率自适应减速(curvature_min 0.4 / max 0.7 / 减速比例 0.5),输出 `linear.x/y` + `angular.z` |
| Costmap Layers | `static_layer` + `intensity_voxel_layer` + `inflation_layer` | 核心为自研 `pb_nav2_costmap_2d::IntensityVoxelLayer`(3D 体素 + intensity 高度过滤, z_voxels=16, z_resolution=0.05) |
| Behaviors | `pb_nav2_behaviors/BackUpFreeSpace`(自研)+ `Spin` + `Wait` + `DriveOnHeading` + `AssistedTeleop` | BackUp 查询全局代价地图 `global_costmap/get_costmap` 服务,向最大自由扇区回退(max_radius=2.0) |
| 地形输入 | `terrain_map`(local, 5m)/ `terrain_map_ext`(global, 10m) | terrain_analysis(_ext) 产出的带 intensity 点云 |
| BT | `RecoveryNode(10)` → `PipelineSequence` → `RateController(3Hz)` → 清 costmap + `RoundRobin`(清代价地图/BackUp) | 标准导航 + 重规划 + 恢复 |

`cmd_vel` remap 链路(launch 验证):

```
controller_server ──cmd_vel_controller──> velocity_smoother ──cmd_vel_nav2_result──> fake_vel_transform ──cmd_vel(+cmd_spin)──> 底盘
```

关键参数:local_costmap 5×5m / 0.05m,global_costmap 用 map 系;`robot_base_frame: gimbal_yaw_fake`(bt_navigator/local/global costmap/behavior_server 均为此);controller 的 `min_y_velocity_threshold: 0.5`(全向机器人 y 向速度阈值特殊)。

### 1.5 启动结构

`pb2025_nav_bringup/launch/`:

| 文件 | 职责 |
| --- | --- |
| `bringup_launch.py` | 顶层:PushRosNamespace + SetRemap(/tf→tf) + nav2_container(composition) + 按 `slam` 选 `slam_launch` 或 `localization_launch`,再含 `navigation_launch` |
| `localization_launch.py` | 非 SLAM:point_lio(带 prior_pcd) + map_server + small_gicp 重定位 + lifecycle_manager_localization |
| `slam_launch.py` | 建图:slam_toolbox(sync) + pointcloud_to_laserscan(terrain_map_ext→obstacle_scan)+ point_lio(pcd_save_en=True)+ 静态 map→odom tf + map_saver |
| `navigation_launch.py` | terrain_analysis(_ext) + loam_interface + sensor_scan_generation + fake_vel_transform + nav2 全栈(composition 双路径) |
| `rm_navigation_simulation_launch.py` | 仿真入口(rmuc/rmul 2024/2025,默认 namespace=red_standard_robot1,use_sim_time=True)+ ign_sim_pointcloud_tool + rviz + joy |
| `rm_navigation_reality_launch.py` | 实车入口(默认 namespace="",use_sim_time=False)+ livox_ros_driver2 + 可选 robot_state_publisher(pb2025_robot_description) |
| `rm_multi_navigation_simulation_launch.py` | 多机器人(实验性) |
| `joy_teleop_launch.py` / `robot_state_publisher_launch.py` / `rviz_launch.py` | 手柄 / TF / RViz |

仿真 vs 实车参数差异(`diff config/simulation vs config/reality` 验证):
- 雷达:仿真 `velodyne_points`(lidar_type=2, scan_line=32)/ ign_sim_pointcloud_tool;实车 `livox/lidar`(lidar_type=1, scan_line=4, timestamp_unit=3 纳秒)+ livox_ros_driver2(mid360_user_config.json)。
- 重力/外参:仿真重力对齐 `[0,-4.9,-8.487]`(rpy=[0,pi/6,0]),实车 `[0,0.5305,-0.8384]` + 非零 extrinsic_T。
- `use_sim_time` 全部 True/False。

---

## 二、项目二:HERO_2026_Sentry_NAV(`HERO_2026_Sentry_NAV/`)

### 2.1 概览

- 团队:哈工大(威海)HERO 竞技机器人实验室 25/26 赛季导航组。
- 目标:实车比赛,模块化分层(决策/感知/规划/接口),算法激进。
- 传感器:Odin1 激光雷达(发布 `/odin1/cloud_slam`、`/odin1/odometry_highfreq`)+ 双 Livox Mid-360。
- 依赖:ACADOS(MPC,`hero_mpc_controller/model/c_generated_code/` 已含生成代码)、BehaviorTree.CPP(vendored 于 `decision/BehaviorTree.CPP`)、ROG-Map(参考)、GCopter/MINCO(`pb_minco_smoother/include/gcopter/` vendored)。
- 工程化:比赛产物(独立 git 仓库,仅一次 commit "Initial commit"),存在硬编码绝对路径(`/home/dji/...`、`/home/z/...`)、`TODO` 描述、注释掉的节点、`.cpp` 备份文件(`fast_Layer copy.*`、`merge.cpp` 旧版)。
- 辅助脚本:`scripts/lidar_extrinsic_calibration.py`(459 行,rosbag2 外参标定:云台旋转轨迹最小二乘拟合圆 + 姿态四元数求平均安装倾角,独立工具)。

**总数据流**(源码验证):

```
双 Livox ──> lidar_merge (/livox/fused_cloud, CustomMsg 融合)
   ──> hero_lidar_scan (/livox/cloud_scans, 360° 稠密 + 逐点去畸变, 用 Odin1 高频里程计)
   ──> dog_map (/rog_map/inf_occ 占用, /rog_map/ground 地面, map系, OccMap 体素栅格)
   ──> fast_layer(代价层) + update_static_layer(静态层) + inflation_layer
   ──> planner(SmacPlannerHybrid) ──> smoother(pb_minco/MincoSmoother
       → /smoother_server/minco_polynomial_trajectory) ──> controller(hero_mpc_controller → cmd_vel)

decision/bt 通过 navigate_to_pose action 驱动 nav2;
nav_cv_bridge 融合自瞄(/tracker/autoaim) + 雷达(/tracker/radar) → /global_targets;
zenoh_bridge.json5: 双机通信,允许 publishers [/carrot_pose,/MaskID,/gimbal_decision], subscribers [/tracker/autoaim]
```

### 2.2 目录结构

```
HERO_2026_Sentry_NAV/
├── decision/             # 决策层:行为树状态机
│   ├── bt/               #   决策节点(基于 BehaviorTree.CPP)
│   │   ├── include/      #     bt_node.hpp / behavior_node.hpp / cruiser_node.hpp /
│   │   │                 #     guard_with_attack_node.hpp / attack_rune_nodes.hpp /
│   │   │                 #     thread_node.hpp / color_info_map.hpp
│   │   ├── src/          #     refree_subscriber_node.cpp(decision_node 入口)、
│   │   │                 #     refree_pub_test.cpp、radar_pub_test.cpp(仿真器)
│   │   ├── config/xml/   #     47 个行为树 XML(RMUC/RMUL 各场次 + 测试)
│   │   ├── config/yaml/  #     decision_node_param.yaml 等
│   │   └── launch/decision_launch.py
│   └── BehaviorTree.CPP/ #   (vendored 三方库,忽略)
├── perception/           # 感知层前端
│   ├── lidar_merge/      #   双雷达 CustomMsg 点云融合
│   ├── hero_lidar_scan/  #   360° 稠密点云 + 运动畸变去除
│   ├── nav_cv_bridge/    #   雷达+视觉自瞄数据融合
│   └── zenoh_bridge.json5
├── planning/             # 规划/后端
│   ├── dog_map/          #   点云后端占用栅格地图(ROG-Map 风格)
│   ├── fast_layer/       #   nav2 costmap 插件层
│   ├── hero_mpc_controller/  # ACADOS MPC 控制器(含 c_generated_code)
│   ├── pb_minco_smoother/    # MINCO 轨迹优化 smoother(gcopter vendored)
│   ├── pb_nav2_plugins/  #   nav2 扩展插件(行为/条件/代价层)
│   └── hero2025_nav_bringup/ # 启动 + nav2_params.yaml
├── interfaces/           # 自定义消息/服务
│   ├── interfaces/       #   GlobalTarget、MincoTrajectory、NavOutput 等
│   └── serial_interfaces/ #  串口/裁判系统 msg + Map.srv
└── scripts/              # lidar_extrinsic_calibration.py 外参标定
```

### 2.3 各包职责

#### 决策层 `decision/bt`

- 基于 **BehaviorTree.CPP**(vendored)的状态机:能量机关状态机、追击、守卫、巡逻、优先级打断。运行时由 `decision_node_param.yaml` 的 `xml_config_path` 指定(默认 `RMUC_26_5_29.xml`)。
- 核心头文件(源码验证):
  - `bt_node.hpp`(322 行)— 基础节点:`CheckValueNode`(黑板值比较 eq/ne/gt/lt/ge/le)、`DoubleCheckValueNode`(带触发/退出双阈值迟滞 + 黑板 flag)、`CheckCountdownNode`、`ControlAutoAim` 等。
  - `behavior_node.hpp`(407 行)— `GuardianNode`、`AttackNavNode`(接近目标 ~2m 提前成功)、`AutoAimManagerNode`(JSON 目标→黑板)、`ClearFlagNode`。
  - `cruiser_node.hpp`(283 行)— `CruiserStateManagerNode`(巡逻状态递增)、`NavNode`(发布 `arrived_at_goal` Bool)。
  - `guard_with_attack_node.hpp`(767 行)— `GuardWithAttackNode`(订阅 `GlobalTargetArray` 敌方目标 + `global_costmap/costmap_raw` OccupancyGrid + `OccupancyGridUpdate`,在最近敌方周围圆上选最佳交战点,发布 MarkerArray 可视化)。
  - `attack_rune_nodes.hpp`(223 行)— `PublishNavOutputNode`(发布 `interfaces/NavOutput`)、`RuneStateMachineNode`(能量机关 20s 激活,发布 Bool + NavOutput)。
  - `thread_node.hpp`(732 行)— 后台线程:`BlackboardUpdater`(订阅裁判系统 `refree_msg`/`radar_msg`/trigger → 黑板)、`hp_spin_watcher`(血量危险发布 Bool)、`GimbalSerialNode`(发布 `TargetSwitchs`)、`sentry_attack_watcher`、`AdjustPositions`。
  - `color_info_map.hpp`(158 行)— `ColorMapModel`(PGM+PNG HSV 红色区→豁免区地图)。
- 可执行:`decision_node`(`refree_subscriber_node.cpp`,注册 12 个节点类型:CheckValue/DoubleCheckValue/LogMessage/CheckCountdown/ControlAutoAim/Nav/CruiserStateManager/Guardian/AttackNav/AutoAimManager/ClearFlagNode/PointInQuadrilateral;BlackboardUpdater + hp_spin_watcher 线程启动,GimbalSerialNode/AdjustPositions/sentry_attack_watcher 被注释)、`refree_pub`、`radar_pub`(后两个为仿真器,launch 中也被注释)。
- 话题:订阅 `refree_msg`/`radar_msg`/`/global_targets`/`/global_costmap/costmap` 等;发布 `/tovision`、`/carrot_pose`、`RuneActivateCommand`、`gimbal_decision` 等;通过 `navigate_to_pose` action 驱动导航。

#### 感知层前端 `perception/`

| 包 | 职责(源码验证) |
| --- | --- |
| `lidar_merge` | 双 Livox `CustomMsg` 融合(T_lidar1_to_lidar2 外参 + 盲区剔除 `isBlind` + TBB 并行),发布 `/livox/fused_cloud`;`TWO_LIDARS` 宏控制同步/单雷达路径 |
| `hero_lidar_scan` | 用 Odin1 高频里程计(`/odin1/odometry_highfreq`)做逐点运动畸变去除,输出 360° 稠密 `PointCloud2`(`/livox/cloud_scans`,外参 pitch≈0.61rad 双雷达侧装);`TWO_LIDARS`/`OMP_LIDAR` 宏均未在 CMakeLists 定义,实际编译单雷达 TBB 路径 |
| `nav_cv_bridge` | 融合视觉自瞄(`/tracker/autoaim` TrackerOutput)与雷达(`/tracker/radar` GlobalTargetArray)→ `/global_targets`(带迟滞切换, radar_switch_dist 5.5 / autoaim_switch_dist 4.5)+ `MaskID` 豁免区标记 |

另有 `zenoh_bridge.json5` — 双机(导航机 + 自瞄机)通信,基于 zenoh-bridge-ros2dds + PTP 时间戳硬同步。

#### 感知后端 `planning/dog_map` + `fast_layer`

- `dog_map`(参考 ROG-Map):局部 3D 占用栅格地图后端。
  - 输入:`/livox/cloud_scans`(360° 雷达)+ `/odin1/cloud_slam`(Odin1,`ODIN_WIDTH_LIDAR` 宏下 message_filters 同步 + 统计离群点滤波)。`FIX_MAP` 宏在 CMakeLists 中被注释(`# add_compile_definitions(FIX_MAP)`),`/rog_map/fix` 不会发布。
  - 数据结构:`HEROPointXYZK`(含 `device_id` 区分雷达)、`OccMap`(扁平 `uint8` 体素,`LOG_OCC_HIT_mid360=34/LOG_OCC_HIT_odin1=16/LOG_OCC_FREE=-20/THR_OCC=30` 对数概率)、`StaticFixMap`(用先验 PGM 做射线修正)。
  - 算法:Amanatides-Woo 3D 射线步进(`racyHandle`,**当前为空 TODO**)+ 命中累计 + 时间遗忘(`TimeGoON`,未观测 >frame_save 帧衰减 +LOG_OCC_FREE/2,<THR_OCC 移除);`InitMap` 按 `layer_z=int(1.0/resolution_z)`(resolution_z=0.01 → 100 层),`occ_map` 为 rows×cols×layer_z uint8(2000×2000×100 ≈ **400MB**);`Clear()` 只清第一层 2D 切片。
  - 发布:`/rog_map/inf_occ`(占用)、`/rog_map/ground`(地面),map 系;1ms 定时器驱动 `updateCallback`(update_lock + unfinished_frame_cnt 防堆积);点云转 map 系时 TF 的 z 加 0.26~0.3m 防越界。
- `fast_layer`:`nav2_costmap_2d::Layer` 插件,订阅 `/rog_map/inf_occ`,把点投影到代价地图标 `LETHAL_OBSTACLE`(每次 update 累积,非持久栅格)。
- `update_static_layer::UpdateStaticLayer`:static_layer 插件(不在本仓库内),订阅 `cloud_update_topic` 动态更新静态层。

#### 规划层 `planning/`

| 组件 | 插件/实现 | 说明(源码验证) |
| --- | --- | --- |
| Global Planner | `nav2_smac_planner/SmacPlannerHybrid` | Hybrid-A*/Dubin(motion_model DUBIN, minimum_turning_radius 0.05, allow_unknown, smooth_path),expected_planner_frequency 20Hz |
| Smoother | `pb_minco/MincoSmoother` | MINCO 轨迹优化(`Trajectory<5,2>` 五次多项式),两阶段:stage1 形状优化(weight_smooth 5/obstacle 8000/feasibility 40/time 20)+ stage2 动力学优化(1/7500/100/25),L-BFGS,ESDF 避障(`CostmapESDFAdapter`,障碍梯度正交投影去纵向拖拽),`findProjectionOnTrajectory` 热启动;发布 `minco_trajectory`(Path 可视化)与 `~/minco_polynomial_trajectory`(MincoTrajectory msg,controller 消费) |
| Controller | `hero_mpc_controller::HeroMpcController` | 100Hz,ACADOS 生成的 6 状态全向动力学 MPC:状态 `[x,y,ψ,vx,vy,ω]`、控制 `[ax,ay,α]`,线性双积分器;N=40、Tf=2.0s、dt=0.05;acados `SQP_RTI` + `PARTIAL_CONDENSING_HPIPM` + `ERK`;weight_q=[800,800,20,100,100,5], weight_r=[1,1,0.5], carrot_point_index=8, enable_yaw_tracking=false;订阅 `/smoother_server/minco_polynomial_trajectory`,采样轨迹求解后取 `x_1` 速度作 cmd_vel,初速度用上一拍 MPC 命令(状态解耦抗抖) |
| Costmap Layers | `update_static_layer` + `fast_layer` + `inflation_layer` | static_layer 由 `cloud_update_topic: /rog_map/fi`(疑为 `/rog_map/fix` 拼写错误)动态更新;fast_layer 订阅 `/rog_map/inf_occ`;local 5×5m/0.025m,global 用 map 系 |
| Behaviors | `pb_nav2_behaviors/BackUpFreeSpace`(自研)+ `Spin` + `Wait` + `DriveOnHeading` | BackUp 用 ESDF 梯度逃逸方向回退(max_speed=1.0, safe_distance_threshold=0.1, ramp_duration=1.0) |
| 卡死检测 | `pb_nav2_plugins::IsStuckCondition`(自研 BT 条件) | 订阅 costmap,footprint 中心 cost≥253 连续 N 次(检查间隔 0.2s)触发回退;注意 TF 查找硬编码 `base_link` |

- `hero_mpc_controller` 详细:configure 声明 trajectory_timeout/max_velocity/max_omega/max_acceleration/max_alpha/enable_yaw_tracking/reference_yaw 等;`computeVelocityCommands` 每控制周期:获取状态 → 检查轨迹有效性 → 计算相对时间 → 在 MINCO 轨迹上采样 N+1 参考点 → 设 MPC 初值与参考 → 求解 → 提取速度指令转机器人系 → 发布 cmd_vel。发布 `FollowPath/predicted_path`、`/reference_path`(MarkerArray)、`/carrot_pose`(NavOutput,0.15m 距离门限被注释);`setSpeedLimit` 未实现(TODO)。生成脚本 `model/omnidirectional_dynamic_tracking.py`(acados Python,与实际 C++ 控制器不同,仅用于生成 c_generated_code)。mpc_wrapper 内部默认权重 Q=[400,100,20,10,10,5]/R=[1,1,0.5]/W_e=[200,200,50,20,20,10](与 nav2_params 注入的 weight_q=[800,800,20,100,100,5] 不同,以 yaml 为准)。
- `pb_minco_smoother` 详细:两阶段优化权重、`integral_resolution=32`、`mean_time_lower/upper_bound 0.9/1.1`、`max_vel 2.2/max_acc 3.0/safe_distance 0.3`、`output_dt 0.1`、`resample_time_resolution 0.65`;`CostmapESDFAdapter` 从 master costmap 快照做有符号 EDT(max_distance = 10×safe_distance, roi_margin=4.0)。决策变量 = [内点 (N-1)×2, 时间 τ=N],时间用 `0.5τ²+τ+1` 多项式映射保证正性;L-BFGS(mem 256);起点 PVA 热启动(投影到上一帧轨迹,<0.25m 继承 PVA);注意 `include/pb_minco_smoother/lbfgs.hpp` 与 `include/gcopter/lbfgs.hpp` 各有一份 L-BFGS(重复实现)。
- `pb_nav2_plugins`(HERO 版)是 root 版 **超集**:多出 `bt_nodes/is_stuck_condition`(自研卡死条件)与 `utils/local_esdf`(局部 ESDF 工具);`intensity_voxel_layer` 两版源码完全相同(diff 无差异);`back_up_free_space` 差异大(root 版:max_radius + visualize 可视化;HERO 版:max_speed + safe_distance_threshold + ramp_duration + ESDF 梯度方向)。

#### 接口 `interfaces/`

- `interfaces`(10 个 msg):`GlobalTarget`(string id + Point position)、`GlobalTargetArray`(GlobalTarget[] targets)、`MincoTrajectory`(trajectory_id/start_time/durations/coefficients)、`NavOutput`(mode:0 导航/1 哨站/2 小符/3 大符/4 禁用 + PoseStamped)、`Gimbaldecision`、`Tovision`、`TargetSwitch`/`TargetSwitchs`、`TrackerOutput`、`MaskID`。
- `serial_interfaces`(10 个 msg + 1 srv):`Refree`(裁判系统全量状态)、`Radar`、`Robot`、`Enemy`、`Judgment`、`Spin`、`SerialReceive`、`Tobase`、`Tojudge`、`Tovision` + `Map.srv`(x,y → height 高程查询)。

#### 启动 `hero2025_nav_bringup`

- `bringup_launch.py`(顶层,与 pb2025 同源,中文注释翻译版)→ `navigation_launch.py` / `rviz_launch.py`;`slam_launch.py`/`localization_launch.py` 被引用但**仓库中不存在**。
- `navigation_launch.py`:启动 `dog_map_node` + nav2 全栈(composition 双路径);`terrain_analysis`/`terrain_analysis_ext` 节点声明存在但被注释掉(318-319 行)。
- `nav2_params.yaml`:帧系 `map→odom→base_link`,里程计 `/odin1/odometry_highfreq`,controller 100Hz,bt_navigator 注册了自研 `pb_is_stuck_condition_bt_node`。
- 行为树 XML:`navigate_to_pose_w_replanning_and_recovery.xml`(`RecoveryNode(150)` → `PipelineSequence` → `RateController(15Hz)` → `SmoothPath(minco_smoother)` + `FollowPath`;恢复:`ReactiveFallback` → `GoalUpdated` + `IsStuckCondition(costmap_raw, 0.2s, 3次)` + `BackUp(1.0m, 1.0m/s)` + 清代价地图 + `Wait 0.2s`)。

---

## 三、两个项目的关键差异(融合关注点)

| 维度 | pb2025(根目录) | HERO |
| --- | --- | --- |
| 里程计/定位 | point_lio + loam_interface + small_gicp 重定位(map→odom) | Odin1 SLAM(`/odin1/cloud_slam`、`/odin1/odometry_highfreq`,Odin1 自带 map→odom) |
| robot_base_frame | `gimbal_yaw_fake`(fake_vel_transform 处理云台自旋) | `base_link`(无云台自旋处理) |
| TF 帧 | map→odom→base_footprint / gimbal_yaw→gimbal_yaw_fake→front_mid360 | map→odom→base_link |
| Global Planner | ThetaStar(w_traversal_cost=10) | SmacPlannerHybrid(Dubin, 容差 1.5m, allow_unknown) |
| Smoother | SimpleSmoother(无 BT 集成) | MINCO(`pb_minco/MincoSmoother`,BT 中 SmoothPath 集成) |
| Controller | 全向 PID + Pure Pursuit(20Hz,enable_rotation=false) | ACADOS MPC(100Hz,6 状态动力学,全状态追踪) |
| 代价地图层 | static + intensity_voxel_layer(自研,3D 体素 intensity 过滤)+ inflation | update_static_layer(外部) + fast_layer(dog_map 后端)+ inflation |
| 地形/障碍 | terrain_analysis(_ext) → 点云 intensity | dog_map → `/rog_map/inf_occ`、`/rog_map/ground` |
| 决策层 | 无(纯 nav2 行为树) | BehaviorTree.CPP 状态机(能量机关/追击/打断,47 个 XML) |
| 感知 | 单雷达 + 地形分析 | 双雷达融合 + 稠密全景 + 视觉融合(zenoh 双机) |
| 命名空间 | 仿真默认 `red_standard_robot1` | 默认空 `""` |
| 仿真支持 | 有(仿真/实车双模式 + slam 建图) | 无(纯实车) |
| BT 恢复策略 | RoundRobin(清地图/BackUp),RateController 3Hz | IsStuckCondition + BackUp,RateController 15Hz,SmoothPath 集成 |
| 工程化 | 高(CI/pre-commit/文档) | 低(硬编码路径、注释残留、备份文件、拼写错误) |

**重叠/同源部分**(融合最省力切入点):
1. `pb_nav2_plugins`(HERO 版 = root 版超集,intensity_voxel_layer 完全一致)。
2. `terrain_analysis` / `terrain_analysis_ext`(HERO 的 navigation_launch 仍保留声明)。
3. nav2 bringup 启动框架、`RewrittenYaml`/`<robot_namespace>` 替换、`cmd_vel` remap 链路、`BackUpFreeSpace` 行为。
4. behavior_trees XML 结构(`navigate_to_pose_w_replanning_and_recovery.xml` 同源)。

---

## 四、融合任务关注点(预判 + 已发现的坑)

### 4.1 架构决策点
1. **定位源统一**:HERO 用 Odin1(自带里程计 + 定位),pb2025 用 point_lio + small_gicp。需决定以哪套为定位源,或做可插拔抽象(两者最终都要发布 `odometry` + `map→odom` TF)。
2. **帧系对齐**:`base_link`(HERO)vs `base_footprint→gimbal_yaw→gimbal_yaw_fake→front_mid360`(pb2025)的取舍,以及 `fake_vel_transform` 云台自旋处理是否保留(注意 `IsStuckCondition` 硬编码 `base_link` 的耦合)。
3. **代价地图层**:`intensity_voxel_layer`(点云 intensity 高度)vs `fast_layer`(dog_map 后端)的取舍或共存;`update_static_layer` 插件缺失问题。
4. **规划/控制栈**:MPC + MINCO(HERO)vs PID + Theta*(pb2025)的移植;controller 频率(100Hz vs 20Hz)、`cmd_vel` remap 链路合并、BT 中 SmoothPath 集成方式。
5. **决策层接入**:HERO 的 `decision/bt` + `interfaces` 如何挂到导航栈(通过 nav2 action `navigate_to_pose`)。
6. **地形/障碍物方案**:terrain_analysis(intensity 编码,2D 代价地图友好)vs dog_map(3D 体素栅格,信息更丰富但依赖 Odin1 定位)。

### 4.2 已知缺陷/缺依赖(融合时必须处理,均为源码验证)

#### 4.2.1 pb2025(根目录)自身缺陷

| 问题 | 位置 | 说明 |
| --- | --- | --- |
| composition remap 不一致 | `pb2025_nav_bringup/launch/navigation_launch.py:315-320` vs `213-225` | 组合模式(默认 `use_composition=True`)下 `bt_navigator` **缺少** `cmd_vel→cmd_vel_nav2_result` remap(非组合模式有)→ BT 内行为(BackUp 等)的 cmd_vel 绕过 fake_vel_transform 直达 `/red_standard_robot1/cmd_vel`,与 fake_vel_transform 发布冲突 |
| SLAM 生命周期不匹配 | `slam_launch.py:40,88` | `lifecycle_nodes=["map_saver"]` 但节点实际名为 `map_saver_server` → lifecycle_manager 无法激活它(README 因此让用户手动 `map_saver_cli`) |
| prior_pcd 未真正启用 | `localization_launch.py:128` + `nav2_params.yaml` `prior_pcd.enable: False` | launch 只注入 `prior_pcd_map_path` 但未改 `enable` → point_lio 先验点云分支实际不生效 |
| 默认参数路径不存在 | `bringup_launch.py:113` | `default_value=.../params/nav2_params.yaml` 不存在(实际在 `config/simulation/`) |
| fake_vel_transform 用系统时钟 | `fake_vel_transform/src/fake_vel_transform.cpp:88,100` | 控制器超时判断用 `rclcpp::Clock().now()`,`use_sim_time=True` 下与仿真时钟脱节;且用 local_plan 时间戳间接对齐 cmd_vel 属已知 hack |
| BackUpFreeSpace 坐标系可疑 | `pb_nav2_plugins/src/behaviors/back_up_free_space.cpp:88-92` | `best_angle` 在全局(代价图)系计算,`twist=cos/sin(angle)*speed` 却直接作机体系 cmd_vel,未做全局→机体系旋转;无安全扇区时退化为 `angle=0`(向前退);`onRun` 重复调 `getCurrentPose`;`robot_radius`/`free_threshold` 声明未使用 |
| IntensityVoxelLayer z 覆盖不足 | `pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp` | `unknown_threshold_` 声明后未使用;z 仅覆盖 0~0.8m(`z_voxels=16 × 0.05`),`max_obstacle_height=2.0` 形同虚设,>0.8m 障碍点被静默丢弃 |
| small_gicp 隐患 | `small_gicp_relocalization/src/small_gicp_relocalization.cpp` | `accumulated_cloud_` 无上限累积;`map→odom` 用 `last_scan_time_+0.1s` 未来时间戳;standalone launch 的 `input_cloud_topic=cloud_registered`(camera_init 系)与 bringup 内 `registered_scan`(odom 系)不一致 |
| 多机器人初始位姿无效 | `rm_multi_navigation_simulation_launch.py` | 传入 `x_pose/y_pose/robot_name` 但目标 launch 未声明;无 map→odom 初始化(README TODO) |
| 手柄 auto_control 抢占脆弱 | `pb_teleop_twist_joy/src/pb_teleop_twist_joy.cpp` | `sendGoalPoseAction` 0.25s 限频,L1 松开会 `cancel_goals_before`;`joy->buttons[5]` 无长度检查 |
| ign_sim_pointcloud_tool 伪时间戳 | `ign_sim_pointcloud_tool/src/point_cloud_converter.cpp` | `time` 用 `(point_id % 1875) * 0.1/1875` 硬编码 10Hz 周期伪造,`intensity` 恒 0,垂直角越界的点被丢弃(仅用于骗过 point_lio 预处理) |

#### 4.2.2 HERO 自身缺陷

| 问题 | 位置 | 说明 |
| --- | --- | --- |
| 头文件缺失 | `decision/bt/include/thread_node.hpp:23` | 引用了 `feasibility_calculate_node.hpp`(不存在),`refree_subscriber_node.cpp:129` 注册其 `PointInQuadrilateralCondition` → **构建必挂** |
| 消息缺类型 | `interfaces/interfaces/msg/TrackerOutput.msg:2`、`Gimbaldecision.msg:8` | 引用未定义的 `Target[]`(仓库中无 Target.msg,`GlobalTargetArray.msg` 用的是已定义的 `GlobalTarget`)→ interfaces 包构建必挂 |
| 硬编码路径 | `decision/bt/CMakeLists.txt:28` | `include_directories(/home/dji/projects/hero2025_sentinel_ws/src/serial/boost_asio/include)` |
| 硬编码路径 | `hero_mpc_controller/CMakeLists.txt:36` | `ACADOS_INSTALL_DIR = ${PROJECT_ROOT}/src/third_party/acados`(依赖工作区布局,acados 未 vendored) |
| 硬编码路径 | `nav_cv_bridge/src/autoaim_tracker.cpp:20-24`、`decision/bt/include/thread_node.hpp:653-654` | `/home/dji/...` 日志/调试路径 |
| 硬编码路径 | `decision/bt/config/yaml/decision_node_param.yaml:3` | `xml_config_path: /home/dji/hero2026_-sentry/src/decision/bt/config/xml/RMUC_26_5_29.xml` |
| 硬编码路径 | `planning/dog_map/config/cfg.yaml`、`hero2025_nav_bringup/config/nav2_params.yaml:19` | `map_yaml_path: /home/dji/hero2026_-sentry/src/planning/map_server/map/map.yaml`(且 map_server 包不在本仓库) |
| 硬编码路径 | `decision/bt/include/color_info_map.hpp`、`nav_cv_bridge/.../color_info_map.hpp`、`autoaim_tracker.cpp` | 调试图写 `/home/z/Downloads/...` 等 |
| 外部插件 | `nav2_params.yaml` static_layer | `update_static_layer::UpdateStaticLayer` 不在仓库内 |
| 话题拼写 | `nav2_params.yaml:135,161` | `/rog_map/fi` 疑为 `/rog_map/fix` 拼写错误;且 **dog_map 的 `FIX_MAP` 宏被注释(`CMakeLists.txt:13`)**,`/rog_map/fix` 根本不会发布 → static_layer 动态更新源缺失 |
| 参数拼写 | `hero2025_nav_bringup/config/nav2_params.yaml:27` | `default_server_timfeout` 应为 `default_server_timeout`(bt_navigator 无法读到该参数) |
| 拼写错误 | `nav_cv_bridge/src/autoaim_tracker.cpp:225` | 日志 `"recieve_eginerr"`(应为 receive);另有 `heighter_than_ground_threshold`(应为 higher)等拼写 |
| 消息文档漂移 | `interfaces/MincoTrajectory.msg` | 注释写三次多项式(8 系数/段),实际代码 `Trajectory<5,2>` 五次多项式(12 系数/段,`minco_smoother.cpp:950` COEFFS_PER_DIM=6) |
| MPC 注释与实现不符 | `hero_mpc_controller/src/hero_mpc_controller.cpp` | `get_next_state()` 返回 `x_3` 但注释写 `x_1`;初速度取上一拍 MPC 命令的机制注释为"状态解耦抗抖" |
| MINCO 时间提取不一致 | `pb_minco_smoother/src/minco_smoother.cpp` | 最优时间提取用 `exp` 与代价求值的前向映射不一致(需融合时核对) |
| 依赖写错 | `pb_minco_smoother/package.xml:41` | `<depend>rmoss_interfaces</depend>` 应为 `interfaces`(HERO 自定义消息包) |
| 配置与源码不符 | `perception/lidar_merge/config/cfg_odin.yaml` | 键名与编译版源码不符(旧配置残留) |
| 坐标系混用 | `perception/hero_lidar_scan/src/hero_lidar_scan_node.cpp` | 单雷达模式(TWO_LIDARS 未定义)下用 `T_lidar2_to_base_` 处理 fused_cloud,输出 `frame_id="odom"` 与点云实际坐标系混用;`getPoseAtTimeLocal` 中 `idx_after==0` 分支注释"使用最早帧"却取 `local_odom_buffer.back()`(应为 front) |
| 死代码/备份 | `decision/bt/config/xml/decision.xml` | 整文件被 XML 注释;`fast_layer` 的 `fast_Layer copy.*`、`lidar_merge` 未编译的 `merge.cpp`(参数名不同:lidar1_to_odin1_* vs lidar1_to_lidar2_*);`dog_map/params_load.hpp` 遗留未用 |
| 重复 L-BFGS 实现 | `pb_minco_smoother/include/pb_minco_smoother/lbfgs.hpp` + `include/gcopter/lbfgs.hpp` | 两份 L-BFGS 拷贝(融合时需去重) |
| 消息重复定义 | `interfaces/msg/Tovision.msg` + `serial_interfaces/msg/Tovision.msg` | 两个包同名消息(设计隐患,引用处需小心) |
| 配置重复键 | `decision/bt/config/yaml/refree_pub_param.yaml` | `current_hp` 重复键(400/600) |
| 构建类型硬编码 | `planning/fast_layer/CMakeLists.txt` | `set(CMAKE_BUILD_TYPE "Debug")` 硬编码 |
| 依赖缺失 | `decision/bt/package.xml` | 缺 map_msgs/OpenCV/yaml-cpp 依赖(CMake 需要但 package.xml 未声明) |
| 编译宏漂移 | `perception/hero_lidar_scan`、`perception/lidar_merge` | `TWO_LIDARS`/`OMP_LIDAR` 用 `#ifdef` 但 CMakeLists 未定义,实际只编单雷达 TBB 路径(hero_lidar_scan)与单雷达路径(lidar_merge) |
| 参数漂移 | `dog_map/config/cfg.yaml` vs `nav2_params.yaml` | cfg.yaml 是旧版(odom_topic `/odometry/filtered`、cloud_topic `cloud_registered`、LOG_OCC 无 mid360/odin1 区分),与 nav2_params.yaml 新版不一致;node.cpp 的 declare 默认值也与两者不同(resolution_z 0.03 vs 0.01,frame_save 10 vs 3) |
| 启动文件缺失 | `hero2025_nav_bringup/launch/bringup_launch.py:160,175` | 引用 `slam_launch.py`/`localization_launch.py`,仓库中不存在 |
| BT XML 缺失 | `hero2025_nav_bringup/config/nav2_params.yaml:30` | 引用 `navigate_through_poses_w_replanning_and_recovery.xml`,仓库 behavior_trees/ 下只有 navigate_to_pose 一个 |
| 硬编码帧 | `pb_nav2_plugins/src/bt_nodes/is_stuck_condition.cpp:108` | TF 查找硬编码 `"base_link"`,若 robot_base_frame 改为 gimbal_yaw_fake 会失效 |
| 编译宏未定义 | `planning/dog_map/node.cpp` | `ODIN_WIDTH_LIDAR` 已定义(双雷达同步路径);`FIX_MAP` 未定义 |

### 4.3 依赖清单(两套合并后需统一)
- ROS2 Humble + NAV2(两套共用)。
- 第三方:ACADOS(MPC,`hero_mpc_controller/model/c_generated_code/` 已含生成 C 代码,但仍需 acados 库)、BehaviorTree.CPP(vendored)、OpenCV + yaml-cpp + nlohmann-json(决策/后端)、TBB/OpenMP、small_gicp、slam_toolbox、GCopter/MINCO(vendored)。
- 传感器驱动:livox_ros_driver2、Odin1(私有)/ point_lio、`pb2025_robot_description`(robot_state_publisher 用,外部仓库)。
- 工程化:命名空间、`use_sim_time`、composition、日志统一。

---

*注:本文档基于对启动文件、`nav2_params.yaml`、`package.xml`、行为树 XML 及关键源码的逐包精读(两套独立深度阅读 + 人工复核);MPC 求解器生成细节、MINCO 优化内部实现、行为树 47 个 XML 的具体逻辑将在融合实施阶段进一步精读。*

---

## 五、融合实施状态(2026-08 更新)

HERO 的 6 大模块(决策层 `bt`、感知后端 `dog_map`/`fast_layer`、MPC `hero_mpc_controller`、MINCO `pb_minco_smoother`、回退/等待策略 `pb_nav2_plugins`、标定工具)已迁入根目录并完成编译验证(**37 包全量 colcon 编译通过**;`hero_mpc_controller` 因 acados 未安装自动跳过,装后启用)。

- 融合方案、文件改动清单、启动方式见 **`FUSION_GUIDE.md`**。
- 启动入口: `rm_navigation_simulation_launch.py` / `rm_navigation_reality_launch.py` 新增 `hero_stack:=True`(默认)切换 HERO 融合栈,`hero_stack:=False` 回退原 pb2025 栈(旧规划器 ThetaStar/PID/SimpleSmoother 保留)。
- 原始 HERO 代码保留于 `HERO_2026_Sentry_NAV/`(已加 `COLCON_IGNORE`,仅作参考)。
