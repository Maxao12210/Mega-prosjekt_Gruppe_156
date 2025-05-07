#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to generated calibration YAML
    camera_info_file = os.path.expanduser('~/.ros/camera_info/8m_web_camera.yaml')
    camera_info_url = f'file://{camera_info_file}'

    return LaunchDescription([
        # 1) v4l2 camera driver node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',  # publishes /image_raw and offers /camera/set_camera_info
            parameters=[{
                'video_device':    '/dev/video1',  # use only /dev/video1
                'pixel_format':    'YUYV',
                'image_size':      [640, 480],
                'output_encoding': 'bgr8',
                'camera_name':     '8m_web_camera',
                'camera_info_url': camera_info_url,
                'qos_overrides': {
                    'image': {
                        'reliability': 'reliable',
                        'history':     'keep_last',
                        'depth':       5,
                    },
                    'camera_info': {
                        'reliability': 'reliable',
                        'history':     'keep_last',
                        'depth':       5,
                    },
                },
            }],
            remappings=[
                # publish the camera frames on exact '/image_raw' topic
                ('image', 'image_raw'),
                # camera info on '/camera/camera_info'
                ('camera_info', 'camera/camera_info'),
            ],
            output='screen',
        ),

        # 2) custom color-detection node
        Node(
            package='cube_pointer_robot',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_topic': '/image_raw',
            }],
            output='screen',
        ),
    ])
