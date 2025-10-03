import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import xacro

pkg_folder = 'modelrobot_pkg_nea'
robot_file = 'robot_car_nea.urdf.xacro'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path + '/model/' + robot_file)

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

    # Process the URDF file
    robot_description_config = xacro.process_file(default_model_path)

    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Launch!
    return LaunchDescription([
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])