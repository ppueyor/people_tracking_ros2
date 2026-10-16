import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('people_rgbd')
    default_params_file = os.path.join(pkg_share, 'config', 'people_rgbd_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock instead of wall clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file for people_rgbd_node'
    )
    # The node internally builds its subscriptions as camera_topic + "/rgb/image_raw"
    # etc. Real camera drivers rarely follow that exact naming (e.g. realsense2_camera
    # uses /camera/color/image_raw + /camera/aligned_depth_to_color/image_raw), so
    # each sub-topic is remapped independently here instead of relying on a shared
    # topic-name prefix.
    declare_rgb_image_topic = DeclareLaunchArgument(
        'rgb_image_topic', default_value='camera/rgb/image_raw',
        description='RGB image topic published by the camera driver'
    )
    declare_depth_image_topic = DeclareLaunchArgument(
        'depth_image_topic', default_value='camera/depth_registered/image_raw',
        description='Depth image topic published by the camera driver'
    )
    declare_rgb_camera_info_topic = DeclareLaunchArgument(
        'rgb_camera_info_topic', default_value='camera/rgb/camera_info',
        description='RGB camera_info topic published by the camera driver'
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='Frame to publish detections in (e.g. map or odom)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rgb_image_topic = LaunchConfiguration('rgb_image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    rgb_camera_info_topic = LaunchConfiguration('rgb_camera_info_topic')
    frame_id = LaunchConfiguration('frame_id')

    people_rgbd_node = Node(
        package='people_rgbd',
        executable='people_rgbd_node',
        name='people_rgbd_node',
        output='screen',
        respawn=True,
        remappings=[
            ('camera/rgb/image_raw', rgb_image_topic),
            ('camera/depth_registered/image_raw', depth_image_topic),
            ('camera/rgb/camera_info', rgb_camera_info_topic),
        ],
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'camera_topic': 'camera',
                'frame_id': frame_id,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_rgb_image_topic,
        declare_depth_image_topic,
        declare_rgb_camera_info_topic,
        declare_frame_id,
        people_rgbd_node,
    ])
