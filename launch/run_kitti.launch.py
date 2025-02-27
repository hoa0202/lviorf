import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    project = LaunchConfiguration('project')
    
    declare_project = DeclareLaunchArgument(
        'project',
        default_value='lviorf'
    )
    
    # 설정 파일 경로
    lidar_param_file = os.path.join(
        get_package_share_directory('lviorf'),
        'config', 'pinhole', 'kitti_params_lidar.yaml'
    )
    
    camera_param_file = os.path.join(
        get_package_share_directory('lviorf'),
        'config', 'pinhole', 'kitti_params_camera.yaml'
    )
    
    # LOAM 모듈 실행
    module_loam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lviorf'), 
                         'launch', 'include', 'module_loam.launch.py')
        ])
    )
    
    # Rviz 모듈 실행
    module_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lviorf'), 
                         'launch', 'include', 'module_rviz.launch.py')
        ])
    )
    
    # 파라미터 노드
    param_node = Node(
        package='lviorf',
        executable='image_projection',
        parameters=[lidar_param_file,
                    {'vins_config_file': camera_param_file}]
    )
    
    return LaunchDescription([
        declare_project,
        param_node,
        module_loam,
        module_rviz
    ])