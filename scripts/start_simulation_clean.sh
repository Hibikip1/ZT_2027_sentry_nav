#!/usr/bin/env bash
# =============================================================================
# 干净启动向导:解决仿真时钟倒退(Detected jump back in time / imu loop back)
#
# 根因:gazebo 仿真多次启动/重启后,残留的 ros_gz_bridge / rmoss 桥进程
#      继续发布 /clock 与传感器话题,与新的 gz sim 实例时间基准不同,
#      导致 use_sim_time 节点看到时间倒退(tf2 清缓存、point_lio 清队列)。
#
# 用法:
#   bash scripts/start_simulation_clean.sh <world>   # 例如 rmuc_2025
#
# 步骤:
#   1. 彻底清理残留仿真/导航进程
#   2. 等待 /clock 就绪且无倒退
#   3. 提示后启动建图/导航 launch
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-rmuc_2025}"

echo "===== 1. 清理残留仿真/导航进程 ====="
python3 "$WS/.colcon/cleanup_all.py" || true
sleep 2

echo "===== 2. 检查残留 /clock 发布者 ====="
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
export ROS_HOME="$WS/.colcon/ros_home"
COUNT=$(ros2 topic info /clock -v 2>/dev/null | grep -c "Publisher count: 1" || echo 0)
echo "  /clock 发布者数量: $COUNT (应为 0, 仿真未启动)"

echo ""
echo "===== 3. 请在新终端启动仿真 ====="
echo "  ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
echo "  等待 Gazebo 界面加载完成,点击左下角'启动'按钮"
echo ""
read -r -p "按回车继续(仿真已启动并点过启动按钮)..." _

echo "===== 4. 等待 /clock 稳定(15 秒) ====="
echo "  (避免启动瞬间时钟跳变)"
sleep 15

echo "===== 5. 检查 /clock 发布者与单调性 ====="
COUNT=$(ros2 topic info /clock -v 2>/dev/null | grep -c "Publisher count: 1" || echo 0)
echo "  /clock 发布者: $COUNT (应为 1)"
if [ "$COUNT" -ne 1 ]; then
  echo "  [警告] /clock 发布者数量异常,可能仍有残留,请重跑本脚本"
fi

echo ""
echo "===== 6. 启动建图(原 pb2025 栈) ====="
echo "  命令:"
echo "  ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=$WORLD slam:=True"
echo ""
echo "  导航(hero 融合栈):"
echo "  ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=$WORLD slam:=False"
echo ""
echo "完成。若仍出现时间倒退,请贴出完整日志。"
