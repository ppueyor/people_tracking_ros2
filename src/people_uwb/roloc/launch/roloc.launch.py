import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roloc')
    default_params_file = os.path.join(pkg_share, 'config', 'roloc_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock instead of wall clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file for roloc_worker/roloc_robot'
    )
    declare_range_topic = DeclareLaunchArgument(
        'range_topic', default_value='/localino_node/anchor/range',
        description='P2PRangeWithPose topic published by the UWB anchor gateway (localino_node)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    range_topic = LaunchConfiguration('range_topic')

    roloc_worker = Node(
        package='roloc',
        executable='roloc_node',
        name='roloc_worker',
        output='screen',
        respawn=True,
        remappings=[('/range', range_topic)],
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    roloc_robot = Node(
        package='roloc',
        executable='roloc_node',
        name='roloc_robot',
        output='screen',
        respawn=True,
        remappings=[('/range', range_topic)],
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_range_topic,
        roloc_worker,
        roloc_robot,
    ])
