from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    package_name = 'E03P02_RI_SMS_NEA'
    
    # Get the path to the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'model',
        'RRP.urdf.xacro'
    ])
    
    # Convert XACRO to URDF command
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
            }]
        ),
    ])