from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start kamera-driveren
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'output_encoding': 'rgb8'
            }],
        ),

        # Start ditt kamera-node
        Node(
            package='cube_pointer_robot',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_topic': '/camera/image_raw'
            }],
        )
    ])
