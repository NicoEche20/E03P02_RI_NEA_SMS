from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False
            }]
        ),
        
        # Joint State Publisher (provides default joint positions)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # Our custom nodes with delays to ensure dependencies are loaded
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='E03P02_RI_SMS_NEA',
                    executable='dxf_parser',
                    name='dxf_parser',
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='E03P02_RI_SMS_NEA',
                    executable='trajectory_planner',
                    name='trajectory_planner',
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='E03P02_RI_SMS_NEA',
                    executable='inverse_kinematics',
                    name='inverse_kinematics',
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='E03P02_RI_SMS_NEA',
                    executable='forward_kinematics',
                    name='forward_kinematics',
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='E03P02_RI_SMS_NEA',
                    executable='joint_state_bridge',
                    name='joint_state_bridge',
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='E03P02_RI_SMS_NEA',
                    executable='path_publisher',
                    name='path_publisher',
                    output='screen'
                )
            ]
        ),
    ])