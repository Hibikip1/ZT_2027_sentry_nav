# HERO 模块融合指南(Fusion Guide)

> 本文档记录 HERO_2026_Sentry_NAV 的 6 大模块融合进 pb2025_sentry_nav 的实现方式、改动清单、启动方法与验证结果。前置阅读:`CODE_STRUCTURE_OVERVIEW.md`。

---

## 一、融合范围

按需求并入 HERO 的 6 个模块,并保留 pb2025 原规划/控制栈作为可回退选项:

| # | 模块 | 迁入包 | 作用 |
| --- | --- | --- | --- |
| 1 | 决策层 | `bt`(decision)+ `interfaces` + `serial_interfaces` + `behaviortree_cpp`(vendored v4) | 能量机关状态机、追击、优先级打断(47 个行为树 XML) |
| 2 | 感知后端 | `dog_map` + `fast_layer` | ROG-Map 风格点云占用栅格 → nav2 costmap 投影 |
| 3 | MPC 控制器 | `hero_mpc_controller` | ACADOS MPC 轨迹全状态追踪(需 acados,可开关) |
| 4 | MINCO 优化器 | `pb_minco_smoother` | ESDF 避障 + 两阶段轨迹优化 + 最近投影点重规划 |
| 5 | 回退/等待策略 | `pb_nav2_plugins`(HERO 版替换 root 版) | `BackUpFreeSpace`(ESDF 梯度回退)+ `IsStuckCondition`(卡死检测)+ BT 中 `Wait` |
| 6 | 外参标定工具 | `scripts/lidar_extrinsic_calibration.py` | 最小二乘圆拟合,标定云台旋转半径与安装倾角 |

**未迁入**(HERO 原版保留在 `HERO_2026_Sentry_NAV/` 作参考,已加 `COLCON_IGNORE` 防止 colcon 重复扫描):`lidar_merge`、`hero_lidar_scan`、`nav_cv_bridge`(感知前端,非需求范围)、`hero2025_nav_bringup`(启动框架由 pb2025_nav_bringup 替代)。

---

## 二、已确认的架构决策(融合方案)

1. **定位源桥接**:dog_map 使用**单雷达模式**(移除 `ODIN_WIDTH_LIDAR` 编译宏,`DOG_MAP_DUAL_LIDAR` 选项默认 OFF),直接消费 pb2025 的 `registered_scan`(loam_interface 输出,odom 系);MPC/MINCO/bt_navigator 的 `odom_topic` 参数指向 pb2025 的 `odometry`(odom→gimbal_yaw)。
2. **帧系**:保留 pb2025 的 `gimbal_yaw_fake` 虚拟系(云台自旋适配);`IsStuckCondition` 的 TF 查找帧参数化为 `robot_base_frame` 端口(默认 `base_link`,融合 BT 中显式设为 `gimbal_yaw_fake`)。
3. **插件重名**:`pb_nav2_plugins` 用 HERO 版整体替换 root 版(从 `.gitmodules` 移除子模块引用;HERO 版为超集:`intensity_voxel_layer` 源码相同、`BackUpFreeSpace` 为 ESDF 增强版、另含 `IsStuckCondition` + `local_esdf`)。
4. **感知层共存**:`nav2_params_hero.yaml` 中 costmap 使用 `fast_layer`(替换 `intensity_voxel_layer`),`terrain_analysis(_ext)` 节点保留启动(原地形方案数据流不破坏);通过 `hero_stack:=False` 回退原 `nav2_params.yaml` 全栈。
5. **决策数据源**:启用 HERO 自带裁判/雷达模拟器 `refree_pub`/`radar_pub`(发布 `refree_msg`、`GlobalTargetArray`),供行为树驱动;实车可关停模拟器(`use_simulators:=False`)对接真实裁判话题。
6. **构建缺陷修复**(授权):删除 `PointInQuadrilateral` 注册(其 XML 引用全部为注释)、补齐 `Target.msg`、static_layer 使用标准 `StaticLayer`、修正 `default_server_timfeout` 等拼写、清理硬编码路径。
7. **ACADOS**:`hero_mpc_controller` 在检测不到 acados 时自动跳过编译(其余模块不受影响);安装见 `scripts/install_acados.sh`。

---

## 三、文件改动清单

### 3.1 迁入的新包(根目录,与 pb2025 包平级)

| 路径 | 说明 | 相对 HERO 原版的修改 |
| --- | --- | --- |
| `interfaces/` | 自定义消息 | **新增 `msg/Target.msg`**(string id + geometry_msgs/Point position,修复 `TrackerOutput`/`Gimbaldecision` 引用的未定义类型);CMakeLists 登记 |
| `serial_interfaces/` | 裁判系统消息 | 无 |
| `behaviortree_cpp/` | BT v4.6.2 vendored | 无(作为 ROS2 包编译,CMake 包名 `behaviortree_cpp`,与系统 `behaviortree_cpp_v3` 共存) |
| `bt/` | 决策层 | 删除 `thread_node.hpp` 缺失 include 与 `PointInQuadrilateral` 注册;删除 CMakeLists 硬编码 include;package.xml 补 map_msgs/OpenCV/yaml-cpp;`decision_node_param.yaml` 的 xml_config_path 改由 launch 注入;`decision_launch.py` 重写(启用模拟器 + use_sim_time);注释调试路径 |
| `dog_map/` | 点云占用栅格后端 | CMakeLists:`ODIN_WIDTH_LIDAR` → `DOG_MAP_DUAL_LIDAR` 选项(默认 OFF 单雷达);`node.cpp`:修复 `frame_save` 未 declare 直接 get 的隐藏崩溃 bug;新增 `tf_z_offset` 参数替代硬编码 z 偏移;cfg.yaml 注释硬编码路径 |
| `fast_layer/` | costmap 插件层 | CMakeLists:去掉硬编码 `CMAKE_BUILD_TYPE "Debug"`;package.xml 补 nav2_costmap_2d/pluginlib/tf2_eigen/PCL 依赖 |
| `hero_mpc_controller/` | ACADOS MPC | CMakeLists:acados 可开关(`HERO_MPC_ENABLE` 选项 + `ACADOS_DIR` 环境变量支持,缺失时跳过);package.xml 补 eigen3_cmake_module |
| `pb_minco_smoother/` | MINCO 优化器 | package.xml:`rmoss_interfaces` → `interfaces`;补 PCL/pcl_conversions/sensor_msgs/OpenCV 依赖 |
| `pb_nav2_plugins/` | nav2 插件(替换 root 版) | `IsStuckCondition`:新增 `robot_base_frame` 端口替代硬编码 `base_link`;保持 **v3 include**(`behaviortree_cpp_v3/`,与 Humble bt_navigator 兼容);package.xml 依赖 `behaviortree_cpp_v3`;CMakeLists 移除 `-Werror` |

### 3.2 修改的 pb2025 原有文件

| 文件 | 修改 |
| --- | --- |
| `.gitmodules` | 移除 `pb_nav2_plugins` 子模块条目 |
| `pb2025_nav_bringup/launch/navigation_launch.py` | 新增 `hero_stack` 参数;`hero_stack=True` 时启动 `dog_map_node` |
| `pb2025_nav_bringup/launch/bringup_launch.py` | 新增 `hero_stack` 参数并透传 |
| `pb2025_nav_bringup/launch/rm_navigation_simulation_launch.py` | 新增 `hero_stack`/`decision_xml` 参数;params_file 默认值按 hero_stack 切换;接入决策层(GroupAction + PushRosNamespace) |
| `pb2025_nav_bringup/launch/rm_navigation_reality_launch.py` | 同上(实车版) |
| `pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery_hero.xml` | 新增:HERO 融合 BT(15Hz 重规划 + `SmoothPath(minco)` + `IsStuckCondition(gimbal_yaw_fake)` + `BackUp` + `Wait`) |

### 3.3 新增配置/脚本

| 文件 | 说明 |
| --- | --- |
| `pb2025_nav_bringup/config/simulation/nav2_params_hero.yaml` | 仿真融合参数:MPC(100Hz)/SmacHybrid/MINCO/fast_layer/dog_map 段 |
| `pb2025_nav_bringup/config/reality/nav2_params_hero.yaml` | 实车融合参数(use_sim_time=False) |
| `scripts/gen_hero_params.py` | hero 参数生成脚本(从原 params 派生,可重复执行) |
| `scripts/install_acados.sh` | acados 安装脚本(安装到 `<ws>/src/third_party/acados`) |
| `scripts/lidar_extrinsic_calibration.py` + `README_calibration.md` | HERO 外参标定工具 |
| `HERO_2026_Sentry_NAV/COLCON_IGNORE` | 标记原始 HERO 目录为参考,不参与 colcon 扫描 |

---

## 四、启动方式

### 4.1 仿真

**建图模式(slam:=True)默认使用原 pb2025 栈**(ThetaStar + PID,建图流程成熟稳定,避免 hero 栈在未知地图上规划超时与 TF 时间戳跳变);**导航模式(slam:=False)默认使用 HERO 融合栈**:

```bash
# 建图(默认原栈;如需 hero 栈建图可显式 hero_stack:=True)
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=True

# 完整融合栈导航(MPC + MINCO + dog_map + fast_layer + 决策层 + 裁判模拟器)
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False

# 决策层指定行为树 XML
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py decision_xml:=RMUC_5_29.xml

# 回退原 pb2025 栈(导航模式)
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py hero_stack:=False
```

### 4.2 实车

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=<YOUR_WORLD> slam:=False use_robot_state_pub:=True
```

### 4.3 决策层独立启动

```bash
ros2 launch bt decision_launch.py xml_config_file:=RMUC_26_5_29.xml use_simulators:=True
```

---

## 五、数据流(融合后)

```
pb2025 定位栈(point_lio → loam_interface → sensor_scan_generation → small_gicp)
  ├─ registered_scan(odom 系) ──> dog_map(单雷达模式) ──> /rog_map/inf_occ(map 系)
  │                                                          ├─> fast_layer(local/global costmap)
  │                                                          └─> (可选)static 先验图修补
  └─ odometry(odom→gimbal_yaw) ──> MPC/MINCO/bt_navigator 的 odom_topic

planner(SmacPlannerHybrid) → smoother(pb_minco, BT 内 SmoothPath) 
  → /smoother_server/minco_polynomial_trajectory → controller(HeroMpcController, 100Hz)
  → cmd_vel_controller → velocity_smoother → cmd_vel_nav2_result 
  → fake_vel_transform(+cmd_spin) → cmd_vel(底盘)

决策层:refree_pub/radar_pub(模拟器) → refree_msg/global_targets → decision_node(行为树)
  → navigate_to_pose(action) → bt_navigator(hero BT XML)
```

---

## 六、验证结果(本机 colcon 编译)

| 包 | 状态 |
| --- | --- |
| interfaces / serial_interfaces / behaviortree_cpp(v4) | ✅ |
| pb_nav2_plugins(HERO 版)/ dog_map / fast_layer | ✅ |
| pb_minco_smoother / bt(decision) | ✅ |
| hero_mpc_controller | ✅ **编译成功(需 acados v0.5.0,见下)** |
| terrain_analysis(_ext)/ loam_interface / sensor_scan_generation / fake_vel_transform / ign_sim_pointcloud_tool / pb2025_sentry_nav | ✅ |
| 子模块包(livox_ros_driver2 / pb_omni_pid_pursuit_controller / pb_teleop_twist_joy / small_gicp_relocalization / pointcloud_to_laserscan / point_lio) | ✅ |
| pb2025_nav_bringup | ✅(依赖包编译后) |

### 6.1 acados 版本要求(重要)

HERO 的 `model/c_generated_code` 需要 **acados v0.5.0**(7 参数 `ocp_nlp_constraints_model_set`/`ocp_nlp_out_set` + `relaxed_ocp_qp_solver_plan` + `ocp_nlp_dims_set_constraints`)。**v0.4.x 及更早版本 API 不匹配,无法编译**。

**注意:即使 API 编译通过,v0.4.x/v0.5.0 库与 HERO 原生成代码存在运行时布局不兼容(MPC 求解器初始化时 SIGSEGV/随机异常)**。必须用 v0.5.0 的 acados_template **重新生成**求解器代码:

```bash
# 1. 安装 acados v0.5.0 到 src/third_party/acados(见 scripts/install_acados.sh, ACADOS_VERSION=v0.5.0)
# 2. 安装 Python 依赖
pip3 install casadi
pip3 install -e src/pb2025_sentry_nav/src/third_party/acados/interfaces/acados_template
# 3. 下载 tera 渲染器(acados_template 首次运行会自动提示;已下载到 acados/bin/t_renderer)
# 4. 重新生成求解器代码(输出到 hero_mpc_controller/model/c_generated_code)
export ACADOS_SOURCE_DIR=.../src/third_party/acados
export LD_LIBRARY_PATH=.../src/third_party/acados/lib:$LD_LIBRARY_PATH
python3 scripts/gen_acados_code.py
# 5. 重编译
colcon build --packages-select hero_mpc_controller --symlink-install
```

> 原 HERO 生成代码备份于 `hero_mpc_controller/model/c_generated_code_hero_orig/`。生成脚本 `scripts/gen_acados_code.py` 可重复执行。

### 6.2 运行时实测(`ros2 launch ... slam:=False`)

在用户环境(`/home/hibikip/ros_ws/install`,gazebo 仿真已开启)下实测结果:

| 项 | 结果 |
| --- | --- |
| launch 解析/包查找 | ✅ 全部节点启动,无 "package not found" |
| 容器稳定性 | ✅ **容器不再崩溃**(修复 acados 兼容 + small_gicp 空点云后) |
| `decision_node` 行为树 | ✅ 运行(GuardWithAttack 初始化、状态机推进、**发起导航目标**) |
| `dog_map_node` | ✅ 初始化(无先验图时空栅格;map 系转换依赖 `map→odom` TF) |
| MPC 控制器 | ✅ 配置成功(acados 求解器初始化正常,100Hz) |
| nav2 全栈 | ✅ **12 个 action 全部激活**(navigate_to_pose/compute_path/smooth/follow/backup/wait 等) |
| 决策层→导航联动 | ✅ 行为树发起目标 → planner 响应(目标超出 costmap 为正常警告) |
| 回退/等待策略 | ✅ `wait` 与 `clear_costmap` 恢复机制触发 |

**实测中发现并修复的问题(按发现顺序)**:
1. 顶层 launch `hero_stack` 声明顺序 → 调整 `ld.add_action` 顺序。
2. `dog_map_node` 空 `map_yaml_path` → `initEmptyMap()` 空栅格分支;`resolution` 未初始化 → 默认 `{0.1}`。
3. `hero_mpc_controller` acados API 不匹配无法编译 → 升级 v0.5.0。
4. **acados v0.5.0 库与 HERO 原生成代码运行时布局不兼容(MPC 初始化 SIGSEGV)→ 用 v0.5.0 的 acados_template 重新生成求解器代码(scripts/gen_acados_code.py)**。
5. **small_gicp_relocalization 对空/极小点云配准 SIGSEGV(先验 PCD 缺失时)→ 增加最少点数防护(kMinRegistrationPoints=100,跳过配准)**。
6. 多次 launch 测试残留大量进程导致话题/容器冲突 → 提供 `.colcon/cleanup_nav.py` 清理脚本。
7. **建图模式(slam:=True)hero 栈下 TF 时间跳变(`Detected jump back in time`)与 planner 超时(`missed deadline`)**:①`fake_vel_transform` 用独立 50Hz 时钟发布 `gimbal_yaw→gimbal_yaw_fake`,与点云 TF 时间戳错乱 → **TF 时间戳改用最新 odometry 时间戳**;②建图时 hero 规划栈(SmacHybrid 在未知地图搜索慢)不必要 → **slam:=True 默认回退原 pb2025 栈**(`hero_stack` 默认值与 `slam` 联动)。

### 6.3 运行前提(实测环境)

- **先验 PCD(导航必需)**:`pb2025_nav_bringup/pcd/simulation/rmuc_2025.pcd` 不存在(git 不存储),需按 README 从 FlowUs 下载放入。缺失时 small_gicp 无全局地图,`map→odom` 不可用,全局代价地图与 dog_map 的 map 系转换受限(导航目标会报 "out of costmap")。
- **仿真器**:需先启动 `rmu_gazebo_simulator`(提供点云与 TF)。
- 无显示环境运行时可加 `use_rviz:=False`。

### 6.5 标点失败(Send goal call failed)与操作时序

`rviz2: Send goal call failed` = `navigate_to_pose` action server **尚未就绪**(bt_navigator 未激活)。触发原因与正确时序:

1. **残留进程导致 DDS 冲突**(最常见):多次启动仿真/导航会堆积 `ros_gz_bridge`/`rmoss`/容器进程,并遗留 `/dev/shm/fastrtps_*` 共享内存,导致 `RTPS_TRANSPORT_SHM` 错误与 action 通信失败。**每次启动前清理**:
   ```bash
   python3 src/pb2025_sentry_nav/.colcon/cleanup_all.py
   # 或手动: pkill -f "ros_gz_bridge"; pkill -f component_container; rm -f /dev/shm/fastrtps_*
   ```
2. **标点过早**:nav2 全栈激活需要时间(依赖 TF/数据)。**标点前确认 action 就绪**:
   ```bash
   ros2 action list | grep navigate_to_pose   # 出现即就绪,再标点
   ```
3. **先验 PCD 缺失**:即使目标发出,全局地图空(small_gicp 无地图)也会规划失败/车不动。下载 `rmuc_2025.pcd` 放入 `pb2025_nav_bringup/pcd/simulation/`。

**推荐时序**:清理 → 启动仿真 → 点启动 → 等 `ros2 action list` 出现 navigate_to_pose → RViz 标点。

### 6.6 速度/云台自旋问题

- **龟速移动**:根因是 `fake_vel_transform` 的 `init_spin_speed` 原为 `3.14`(哨兵"底盘固定自旋"设计),但仓库内 **无 `cmd_spin` 发布者** → 底盘被强制叠加 3.14 rad/s 旋转(转圈)→ 前进被干扰 → 龟速。已改为 `0.0`(仿真/实车 4 个 params)。若需要底盘自旋扫描,请提供 `cmd_spin` 发布者(报告实际自旋速度)。
- **云台不转**:云台(gimbal 关节)自旋由**云台控制系统**驱动(`cmd_gimbal_joint`/`GimbalCmd`,来自手柄 `pb_teleop_twist_joy`、自瞄系统或自定义脚本),**导航层不控制云台**。融合未改动云台控制;恢复原版云台自旋请使用你原版的方式(手柄/脚本/自瞄)。测试可手动发布:
  ```bash
  ros2 topic pub -r 10 /red_standard_robot1/cmd_gimbal_joint sensor_msgs/msg/JointState \
    "{name: [gimbal_yaw, gimbal_pitch], position: [3.14, 0.0]}"
  ```

### 6.4 关于 "时间倒退" 日志(重要)

以下日志是**仿真时钟跳变时的防护性输出**,节点会自动恢复,但会短暂影响导航(需要重新等 TF):

```
robot_state_publisher: Moved backwards in time, re-publishing joint transforms!
pointlio_mapping: imu loop back, clear deque
tf2_buffer: Detected jump back in time. Clearing TF buffer.
```

**触发原因与规避**:
1. **gazebo 仿真重启/世界重载** → `/clock` 重置 → 所有 `use_sim_time` 节点看到时间倒退。**先启动 gazebo 并等世界加载完,再启动导航 launch;不要中途重启 gazebo 而不重启导航。**
2. **导航 launch 重复启动**(旧实例未停干净)→ 新旧节点时间基准混用。重启导航前请清理:`python3 src/pb2025_sentry_nav/.colcon/cleanup_all.py`(含 gz/ros_gz_bridge 残留)或 `pkill -f component_container`。
3. **`fake_vel_transform` 旧版缺陷(重要)**:旧版用独立 50Hz 时钟发布 `gimbal_yaw→gimbal_yaw_fake` TF,与点云/里程计 TF 时间轴错乱 → `tf2 jump back`。已修复:TF 时间戳改用最新 odometry 时间戳。**确认用户 `install` 中的修复已生效**(`stat -L install/fake_vel_transform/lib/*.so` 时间应为最新;install 下为符号链接,`stat` 不带 `-L` 看到的是链接创建时间 10:48,勿误判)。
4. gazebo 暂停/恢复或 RTF 剧烈波动。

已确认:干净启动(gazebo 先行 → 导航 launch)后,发导航目标全程 **0 次时间倒退**(实测)。若频繁出现,按上述 1/2/3 检查,并确认所有修复包已安装(见下)。

编译命令(需在工作区根 `/home/hibikip/ros_ws` 执行;输出目录重定向到仓库内 `.colcon/`):

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_ws
colcon --log-base src/pb2025_sentry_nav/.colcon/log build \
  --symlink-install \
  --build-base src/pb2025_sentry_nav/.colcon/build \
  --install-base src/pb2025_sentry_nav/.colcon/install
```

> 注:完整 workspace 编译需系统依赖 `small_gicp`(已装)、`rmu_gazebo_simulator` 等;`.colcon/` 目录为本地编译产物,建议加入 `.gitignore`。

---

## 七、已知限制与后续工作

1. **acados 未安装**:`hero_mpc_controller` 当前跳过编译。安装:`bash scripts/install_acados.sh`,然后重编译该包;也可 `export ACADOS_DIR=...`。
2. **决策层实车数据源**:当前用 HERO 模拟器(`refree_pub`/`radar_pub`)。实车需按裁判系统协议替换(消息结构见 `serial_interfaces`),或保留模拟器调试。
3. **dog_map 内存**:`OccMap` 初始化 rows×cols×100 层(uint8)≈ 400MB(2000×2000×100),实车前确认内存余量;`resolution_z` 与 `half_map_size` 可调小。
4. **dog_map `racyHandle`(Amanatides-Woo 射线)当前为空 TODO**:占用更新仅靠命中点累计 + 时间遗忘,`LOG_OCC_FREE` 射线清空未生效——影响动态障碍清除精度,融合后建议优先实现。
5. **fast_layer 无历史衰减**:障碍点仅标记当帧(`plists` 每周期清空),历史障碍不累积;若需持久障碍请与 `intensity_voxel_layer` 或 static 层配合。
6. **MPC/MINCO 在 `gimbal_yaw_fake` 系下为实车验证项**:纯平移追踪理论上兼容,但需实车/仿真联调确认轨迹跟踪与自旋叠加的交互。
7. **hero 参数与行为树**:`default_nav_through_poses_bt_xml` 指向的文件未在仓库(仅 navigate_to_pose 用),多航点导航需补充;`IsStuckCondition` 的 `costmap_topic` 在命名空间下为相对话题(launch 已验证 remap 正确)。
8. **点云/里程计时间戳**:dog_map 用 `tf2::TimePointZero`(最新 TF)做 map 系变换,高频运动下有轻微延迟;如需精确同步可改用消息时间戳。
9. **原 pb2025 缺陷(未在本次融合范围)**:composition 模式 `bt_navigator` 缺 `cmd_vel` remap(hero 模式下 BT 的 BackUp 输出将直接发往 `cmd_vel`,与 fake_vel_transform 冲突)——建议后续修复(两处 remap 保持一致)。

---

*本文档随融合实现持续更新;编译/运行结果以实际环境为准。*
