from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    box_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('camera_node_package'),
                'launch',
                'box_detection.launch.py')))
    
    robot_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_moveit_package'),
                'launch',
                'robot_moveit.launch.py'
            )
        )
    )
    return LaunchDescription([box_detection_launch, robot_moveit_launch])
