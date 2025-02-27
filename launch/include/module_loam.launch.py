from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # LIO 관련 노드 실행
    image_projection_node = Node(
        package='lviorf',
        executable='image_projection',
        name='image_projection',
        output='screen'
    )
    
    map_optimization_node = Node(
        package='lviorf',
        executable='map_optimization',
        name='map_optimization',
        output='screen'
    )
    
    imu_preintegration_node = Node(
        package='lviorf',
        executable='imu_preintegration',
        name='imu_preintegration',
        output='screen'
    )
    
    # VIO 관련 노드 실행
    feature_tracker_node = Node(
        package='lviorf',
        executable='feature_tracker',
        name='feature_tracker',
        output='screen'
    )
    
    visual_odometry_node = Node(
        package='lviorf',
        executable='visual_odometry',
        name='visual_odometry',
        output='screen'
    )
    
    return LaunchDescription([
        image_projection_node,
        map_optimization_node,
        imu_preintegration_node,
        feature_tracker_node,
        visual_odometry_node
    ])
