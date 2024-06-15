from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    transform1 = Node( 
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments = ['--x', '1', '--y', '0', '--z', '0', '--yaw', '3.141592', '--pitch', '0', '--roll', '0', '--frame-id', 'link0', '--child-frame-id', 'link1'] 
    ) 
    transform2 = Node( 
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments = ['--x', '0', '--y', '1', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '3.141592', '--frame-id', 'link1', '--child-frame-id', 'link2'] 
    ) 
    transform3 = Node( 
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments = ['--x', '-1', '--y', '1', '--z', '1', '--yaw', '3.141592', '--pitch', '3.141592', '--roll', '3.141592', '--frame-id', 'link2', '--child-frame-id', 'link3'] 
    ) 

    ld = LaunchDescription() 
    ld.add_action(transform1) 
    ld.add_action(transform2) 
    ld.add_action(transform3) 
    return ld 