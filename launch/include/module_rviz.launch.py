from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    project = LaunchConfiguration('project')
    
    declare_project = DeclareLaunchArgument(
        'project',
        default_value='lviorf'
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('lviorf'),
        'launch', 'include', 'config', 'rviz.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=[project, '_rviz'],
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        declare_project,
        rviz_node
    ])