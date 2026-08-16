from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='bt',
        #     executable='refree_pub',
        #     name='refree_pub',
        #     parameters=[
        #         os.path.join(
        #             get_package_share_directory('bt'),
        #             'config/yaml/refree_pub_param.yaml'
        #         )
        #     ]
        # ),
        # Node(
        #     package='bt',
        #     executable='radar_pub',
        #     name='radar_pub',
        #     parameters=[
        #         os.path.join(
        #             get_package_share_directory('bt'),
        #             'config/yaml/radar_pub_param.yaml'
        #         )
        #     ]
        # ),
        Node(
            package='bt',
            executable='decision_node',
            name='decision_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('bt'),
                    'config/yaml/decision_node_param.yaml'
                )
            ]
        )
    ])