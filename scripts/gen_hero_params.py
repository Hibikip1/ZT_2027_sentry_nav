#!/usr/bin/env python3
"""基于原 nav2_params.yaml 生成 hero 融合版(替换规划/控制栈 + 增加 dog_map/fast_layer)

用法: python3 scripts/gen_hero_params.py <src_params.yaml> <dst_params_hero.yaml>
"""
import re
import sys


def top_level_keys(lines):
    keys = []
    for i, line in enumerate(lines):
        m = re.match(r'^([A-Za-z0-9_]+):', line)
        if m:
            keys.append((m.group(1), i))
    return keys


def replace_section(lines, name, new_text):
    keys = top_level_keys(lines)
    idx = None
    for k, i in keys:
        if k == name:
            idx = i
            break
    if idx is None:
        raise SystemExit(f"section {name} not found")
    end = len(lines)
    for k, i in keys:
        if i > idx:
            end = i
            break
    # 保留键行 + 后续结构前缀行(纯键行,如 ros__parameters:/local_costmap:)
    prefix = [lines[idx]]
    j = idx + 1
    while j < end and re.match(r'^\s+[A-Za-z0-9_]+:$', lines[j]):
        prefix.append(lines[j])
        j += 1
    new_lines = new_text.splitlines()
    # 注意:必须保留 lines[:idx](键行之前的所有内容)
    return lines[:idx] + prefix + new_lines + lines[end:]


def append_section(lines, new_text):
    while lines and lines[-1].strip() == '':
        lines.pop()
    return lines + [''] + new_text.splitlines() + ['']


def gen(src, dst):
    with open(src) as f:
        lines = f.read().splitlines()

    # 1. controller_server → HERO MPC(替换内容缩进 4 空格,置于 ros__parameters: 下)
    ctrl = '''    use_sim_time: True
    odom_topic: odometry
    controller_frequency: 100.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 6.28
    FollowPath:
      plugin: "hero_mpc_controller::HeroMpcController"
      max_velocity: 2.2
      max_omega: 1.5
      max_acceleration: 3.0
      max_alpha: 1.0
      enable_yaw_tracking: false
      reference_yaw: 0.0
      trajectory_timeout: 1.0
      weight_q: [800.0, 800.0, 20.0, 100.0, 100.0, 5.0]
      weight_r: [1.0, 1.0, 0.5]
      carrot_point_index: 8'''
    lines = replace_section(lines, 'controller_server', ctrl)

    # 2. planner_server → SmacPlannerHybrid
    planner = '''    use_sim_time: True
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 1.5
      allow_unknown: true
      downsample_costmap: false
      downsampling_factor: 1
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 3.5
      cost_travel_multiplier: 2.0
      motion_model_for_search: "DUBIN"
      angle_quantization_bins: 64
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.05
      retrospective_penalty: 0.025
      reverse_penalty: 1.0
      change_penalty: 0.0
      non_straight_penalty: 1.20
      cost_penalty: 6.0
      rotation_penalty: 5.0
      lookup_table_size: 20.0
      cache_obstacle_heuristic: True
      allow_reverse_expansion: False
      smooth_path: True
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true'''
    lines = replace_section(lines, 'planner_server', planner)

    # 3. smoother_server → MINCO
    smoother = '''    use_sim_time: True
    smoother_plugins: ["minco_smoother"]
    minco_smoother:
      plugin: "pb_minco/MincoSmoother"
      odom_topic: "odometry"
      stage1_weight_smooth: 5.0
      stage1_weight_obstacle: 8000.0
      stage1_weight_feasibility: 40.0
      stage1_weight_time: 20.0
      stage1_weight_mean_time: 40.0
      stage1_max_iterations: 8000
      weight_smooth: 1.0
      weight_obstacle: 7500.0
      weight_feasibility: 100.0
      weight_time: 25.0
      weight_mean_time: 100.0
      max_iterations: 8000
      g_epsilon: 0.0
      integral_resolution: 32
      mean_time_lower_bound: 0.9
      mean_time_upper_bound: 1.1
      max_vel: 2.2
      max_acc: 3.0
      safe_distance: 0.3
      min_waypoints: 2
      output_dt: 0.1
      publish_trajectory: true
      resample_time_resolution: 0.65
      rotation_penalty_weight: 0.1
      dense_sample_resolution: 0.1'''
    lines = replace_section(lines, 'smoother_server', smoother)

    # 4. bt_navigator → hero BT + pb_is_stuck_condition_bt_node
    bt = '''    use_sim_time: True
    global_frame: map
    robot_base_frame: gimbal_yaw_fake
    odom_topic: odometry
    bt_loop_duration: 10
    default_server_timeout: 100
    wait_for_service_timeout: 1000
    default_nav_to_pose_bt_xml: $(find-pkg-share pb2025_nav_bringup)/behavior_trees/navigate_to_pose_w_replanning_and_recovery_hero.xml
    default_nav_through_poses_bt_xml: $(find-pkg-share pb2025_nav_bringup)/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - pb_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node'''
    lines = replace_section(lines, 'bt_navigator', bt)

    # 5/6. local/global costmap: fast_layer 替换 intensity_voxel_layer
    costmap_local = '''      use_sim_time: True
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: gimbal_yaw_fake
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.2
      plugins: ["static_layer", "fast_layer", "inflation_layer"]
      fast_layer:
        plugin: fast_layer::FastLayer
        pointcloud_topic: <robot_namespace>/rog_map/inf_occ
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.7
      always_send_full_costmap: False'''
    lines = replace_section(lines, 'local_costmap', costmap_local)

    costmap_global = '''      use_sim_time: True
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: gimbal_yaw_fake
      robot_radius: 0.2
      resolution: 0.05
      plugins: ["static_layer", "fast_layer", "inflation_layer"]
      fast_layer:
        plugin: fast_layer::FastLayer
        pointcloud_topic: <robot_namespace>/rog_map/inf_occ
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.7
      always_send_full_costmap: False'''
    lines = replace_section(lines, 'global_costmap', costmap_global)

    # 7. 追加 dog_map_node 段
    dogmap = '''dog_map_node:
  ros__parameters:
    use_sim_time: True
    # 单雷达模式(pb2025 融合):直接消费 loam_interface 输出的 odom 系稠密点云
    cloud_topic: "registered_scan"
    odin_topic: "/odin1/cloud_slam"
    half_map_size: 10.0
    resolution_z: 0.01
    heighter_than_ground_threshold: 0.091
    higher_than_car_threshold: 0.7
    frame_save: 3
    LOG_OCC_HIT_mid360: 34
    LOG_OCC_HIT_odin1: 16
    LOG_OCC_FREE: -20
    THR_OCC: 30
    MAX_LOG_MID360: 34
    MAX_LOG_ODIN1: 34
    MIN_LOG: 5
    # 先验静态图(可选,融合默认不启用;需要时注入 map_server 的 yaml)
    map_yaml_path: ""
    # map 系 z 偏移(原 HERO 车体 hack,默认 0.0)
    tf_z_offset: 0.0'''
    lines = append_section(lines, dogmap)

    with open(dst, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"generated {dst}")


if __name__ == '__main__':
    gen(sys.argv[1], sys.argv[2])
