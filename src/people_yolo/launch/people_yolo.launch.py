import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('people_yolo')
    default_params_file = os.path.join(pkg_share, 'config', 'people_yolo_params.yaml')
    default_model_config = os.path.join(pkg_share, 'model', 'yolov4-tiny.cfg')
    default_model_weights = os.path.join(pkg_share, 'model', 'yolov4-tiny.weights')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock instead of wall clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file for yolo_detector_node'
    )
    declare_model_config = DeclareLaunchArgument(
        'model_config', default_value=default_model_config,
        description='Path to the YOLO .cfg file'
    )
    declare_model_weights = DeclareLaunchArgument(
        'model_weights', default_value=default_model_weights,
        description='Path to the YOLO .weights file'
    )
    # Camera topics (remapped to the node-internal image/depth/camera_info names).
    declare_rgb_image_topic = DeclareLaunchArgument(
        'rgb_image_topic', default_value='camera/rgb/image_raw',
        description='RGB image topic published by the camera driver'
    )
    declare_depth_image_topic = DeclareLaunchArgument(
        'depth_image_topic', default_value='camera/depth_registered/image_raw',
        description='Depth image topic published by the camera driver'
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic', default_value='camera/rgb/camera_info',
        description='RGB camera_info topic published by the camera driver'
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='Frame to publish detections in (e.g. map or odom)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    model_config = LaunchConfiguration('model_config')
    model_weights = LaunchConfiguration('model_weights')
    rgb_image_topic = LaunchConfiguration('rgb_image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    frame_id = LaunchConfiguration('frame_id')

    yolo_node = Node(
        package='people_yolo',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        respawn=True,
        remappings=[
            ('image', rgb_image_topic),
            ('depth', depth_image_topic),
            ('camera_info', camera_info_topic),
        ],
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'model_config': model_config,
                'model_weights': model_weights,
                'frame_id': frame_id,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_model_config,
        declare_model_weights,
        declare_rgb_image_topic,
        declare_depth_image_topic,
        declare_camera_info_topic,
        declare_frame_id,
        yolo_node,
    ])
