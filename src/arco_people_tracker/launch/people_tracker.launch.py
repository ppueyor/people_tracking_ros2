import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arco_people_tracker')
    default_params_file = os.path.join(pkg_share, 'config', 'people_tracker_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock instead of wall clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file for people_tracker_node'
    )
    declare_detections_topic = DeclareLaunchArgument(
        'detections_topic', default_value='/people_detections',
        description='Input topic where laser/RGBD/UWB detectors publish people_msgs/People'
    )
    declare_people_topic = DeclareLaunchArgument(
        'people_topic', default_value='/people',
        description='Output topic for fused tracks. Override this if something else in '
                    'your system (e.g. a simulator ground-truth publisher) already uses /people.'
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='Frame to publish fused tracks in (e.g. map or odom). Must match '
                    'the frame the detectors publish in.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    detections_topic = LaunchConfiguration('detections_topic')
    people_topic = LaunchConfiguration('people_topic')
    frame_id = LaunchConfiguration('frame_id')

    people_tracker_node = Node(
        package='arco_people_tracker',
        executable='people_tracker_node',
        name='people_tracker_node',
        output='screen',
        respawn=True,
        remappings=[
            ('/people_detections', detections_topic),
            ('/people', people_topic),
        ],
        parameters=[params_file, {'use_sim_time': use_sim_time, 'frame_id': frame_id}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_detections_topic,
        declare_people_topic,
        declare_frame_id,
        people_tracker_node,
    ])
