import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bt_dir = get_package_share_directory('bt')

    # 行为树 XML 文件名(默认用赛季正式场次 RMUC_26_5_29)
    declare_xml_config_cmd = DeclareLaunchArgument(
        'xml_config_file',
        default_value='RMUC_26_5_29.xml',
        description='Behavior tree XML file name under bt/config/xml',
    )

    declare_use_simulators_cmd = DeclareLaunchArgument(
        'use_simulators',
        default_value='True',
        description='Launch refree_pub/radar_pub referee & radar simulators',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if True',
    )

    use_simulators = LaunchConfiguration('use_simulators')
    use_sim_time = LaunchConfiguration('use_sim_time')
    xml_config_path = PathJoinSubstitution(
        [bt_dir, 'config', 'xml', LaunchConfiguration('xml_config_file')])

    # 裁判系统模拟器(无实车裁判数据时提供数据源)
    refree_pub_cmd = Node(
        package='bt',
        executable='refree_pub',
        name='refree_pub',
        parameters=[
            os.path.join(bt_dir, 'config', 'yaml', 'refree_pub_param.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(use_simulators),
    )

    radar_pub_cmd = Node(
        package='bt',
        executable='radar_pub',
        name='radar_pub',
        parameters=[
            os.path.join(bt_dir, 'config', 'yaml', 'radar_pub_param.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(use_simulators),
    )

    # 决策主节点(行为树状态机)
    decision_node_cmd = Node(
        package='bt',
        executable='decision_node',
        name='decision_node',
        parameters=[
            os.path.join(bt_dir, 'config', 'yaml', 'decision_node_param.yaml'),
            {'xml_config_path': xml_config_path},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # 区域管理器: 读取 zones.yaml, 发布 rviz 可视化 + zone_info 给决策层
    zone_manager_cmd = Node(
        package='bt',
        executable='zone_manager',
        name='zone_manager',
        parameters=[
            {'zones_config_path': os.path.join(bt_dir, 'config', 'yaml', 'zones.yaml')},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_xml_config_cmd,
        declare_use_simulators_cmd,
        declare_use_sim_time_cmd,
        refree_pub_cmd,
        radar_pub_cmd,
        decision_node_cmd,
        zone_manager_cmd,
    ])
