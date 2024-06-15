import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    line_follower = Node(
        package= 'final_challenge',
        executable= 'line_follower',
        output = 'screen',
    )

    states = Node(
        package= 'final_challenge',
        executable= 'states',
        output = 'screen',
    )

    ld = LaunchDescription([line_follower, states])
    return ld