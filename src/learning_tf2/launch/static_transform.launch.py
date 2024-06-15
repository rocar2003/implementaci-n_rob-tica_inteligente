from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    transform1 = Node( 
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments = ['--x', '1', '--y', '0', '--z', '0', '--yaw', '3.141592', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'link1'] 
    ) 

    ld = LaunchDescription() 
    ld.add_action(transform1) 
    return ld 