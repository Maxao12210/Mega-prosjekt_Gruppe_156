from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create and return the LaunchDescription object
    
     robot_moveit_node = Node(   
            package='<robot_moveit_package>',
            executable='<robot_moveit.cpp>',
            name='<robot_moveit>',
            namespace='<namespace>',  
            output='screen',          
            parameters=[{'param_name': 'param_value'}],  # change based on the coordinates taken from camera files
            remappings=[('/old_topic', '/new_topic')]    # 
        ),
    
    
    return LaunchDescription([
   robot_moveit_launch
    ])
