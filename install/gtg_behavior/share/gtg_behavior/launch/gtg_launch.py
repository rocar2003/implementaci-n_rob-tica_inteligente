import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('gtg_behavior'),
            'config',
            'params.yaml'
    )

    generation_node = Node(
        package= 'gtg_behavior',
        executable= 'go_to_goal',
        output = 'screen',
        parameters= [{config}]
    )

    ld = LaunchDescription([generation_node])
    return ld
