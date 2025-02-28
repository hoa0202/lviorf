from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	# 프로젝트 이름 인자 선언
	project_arg = DeclareLaunchArgument(
		'project',
		default_value='lviorf',
		description='Project name'
	)

	# xacro 파일 경로 설정
	xacro_path = PathJoinSubstitution([
		FindPackageShare('lviorf'),
		'launch/include/config/robot.urdf.xacro'
	])

	# robot_description 파라미터 설정
	robot_description = Command(['xacro ', xacro_path])

	# robot_state_publisher 노드 설정
	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[{
			'robot_description': robot_description,
			'use_sim_time': False
		}],
		respawn=True
	)

	return LaunchDescription([
		project_arg,
		robot_state_publisher_node
	])