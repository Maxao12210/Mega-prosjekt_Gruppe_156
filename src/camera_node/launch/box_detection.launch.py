from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    DeclareLaunchArgument('video', default_value='/dev/video1'),
    # 1) USB camera driver
    Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': LaunchConfiguration('video'),
            'image_size': [1280, 720],
            'output_encoding': 'bgr8',
            'fps': 15,
        }],
    ),


    # 2) Box detection node (color‐box finder)
    Node(
        package='camera_node',
        executable='box_detection_node',
        name='box_detection_node',
        parameters=[{
            'camera_topic': '/image_raw'
        }],
    ),

    # 2) Debugging for mask
    Node(
        package='camera_node',
        executable='camera_debug_node',
        name='camera_debug_node',
        parameters=[{
            'camera_topic': '/image_raw'
        }],
    ),

    # 3) Starts coordinator
    Node(
        package='camera_node',
        executable='coordinator_test_node',
        name='coordinator_test_node',

    ),


    ])
