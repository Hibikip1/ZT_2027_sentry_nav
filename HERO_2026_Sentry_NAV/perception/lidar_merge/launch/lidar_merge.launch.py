import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'lidar_merge'
    
    # 获取参数文件路径
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'cfg.yaml'
    )

    lidar_merge_node = Node(
        package=package_name,
        executable='lidar_merge_node',
        name='lidar_merge_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        lidar_merge_node
    ])