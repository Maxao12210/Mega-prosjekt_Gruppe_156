from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create and return the LaunchDescription object
    
     robot_moveit_node= Node(
            package='robot_moveit',
            executable='robot_moveit',
            name='robot_moveit',
            output='screen',          
            parameters=[{'robot_description': '/robot_description'}],  # change based on the coordinates taken from camera files
            #remappings=[('/old_topic', '/new_topic')]    #
        ),
    
    
     return LaunchDescription(robot_moveit_node)
