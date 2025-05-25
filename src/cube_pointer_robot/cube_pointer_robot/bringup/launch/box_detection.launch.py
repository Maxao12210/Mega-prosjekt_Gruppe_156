from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    # 1) USB camera driver
    Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video4',
            'image_size': [640, 480],
            'output_encoding': 'bgr8',
            'fps': 10,
        }],
    ),

    # 2) Box detection node (color‐box finder)
    Node(
        package='cube_pointer_robot',
        executable='box_detection_node',
        name='box_detection_node',
        parameters=[{
            'camera_topic': '/image_raw'
        }],
    ),

    Node(
        package='cube_pointer_robot',
        executable='coordinator_test_node',
        name='coordinator_test_node',

    ),


    ])