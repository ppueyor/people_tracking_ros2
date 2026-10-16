import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('people_laser_tracker')
    default_hypotheses_file = os.path.join(pkg_share, 'training_data', 'hypo3.dat')
    default_params_file = os.path.join(pkg_share, 'config', 'laser_tracker_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock instead of wall clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file for laser_tracker_node'
    )
    declare_hypotheses_filename = DeclareLaunchArgument(
        'hypotheses_filename', default_value=default_hypotheses_file,
        description='Path to the trained AdaBoost hypotheses file'
    )
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic', default_value='scan',
        description='LaserScan topic to subscribe to (remapped from the node-internal /scan)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    hypotheses_filename = LaunchConfiguration('hypotheses_filename')
    scan_topic = LaunchConfiguration('scan_topic')

    laser_tracker_node = Node(
        package='people_laser_tracker',
        executable='laser_tracker_node',
        name='laser_tracker_node',
        output='screen',
        respawn=True,
        remappings=[('/scan', scan_topic)],
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'hypotheses_filename': hypotheses_filename,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_hypotheses_filename,
        declare_scan_topic,
        laser_tracker_node,
    ])
