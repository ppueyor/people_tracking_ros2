import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('localino')
    default_params_file = os.path.join(pkg_share, 'config', 'localino_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock instead of wall clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file for localino_node '
                    '(this is where the real anchor_list positions live)'
    )
    declare_port = DeclareLaunchArgument(
        'port', default_value='10000',
        description='UDP port the anchor gateway sends range reports to'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')

    localino_node = Node(
        package='localino',
        executable='localino_node',
        name='localino_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'port': port,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_port,
        localino_node,
    ])
