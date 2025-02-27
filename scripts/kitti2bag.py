#!/usr/bin/env python3
# 기존 ROS1용 파이썬 스크립트를 ROS2용으로 변경

import os
import sys
import time
import datetime
import argparse
import numpy as np
from scipy import misc

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2, NavSatFix, Image
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from nav_msgs.msg import Odometry

import rosbag2_py
from rcl_interfaces.msg import ParameterDescriptor

def save_imu_data(bag, kitti, imu_frame_id, topic):
    # IMU 데이터 처리...
    # ROS2 rosbag API를 사용하여 데이터 저장
    pass

def save_dynamic_tf(bag, kitti, tf_frame_id, tf_child_frame_id, topic):
    # 동적 TF 데이터 처리...
    pass

def save_camera_data(bag, kitti, camera_frame_id, topic):
    # 카메라 데이터 처리...
    pass

def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    # GPS 데이터 처리...
    pass

def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    # GPS 속도 데이터 처리...
    pass

def save_point_cloud_data(bag, kitti, velo_frame_id, topic):
    # 포인트 클라우드 데이터 처리...
    pass

def main():
    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS2 bag file.")
    # 명령행 인자 설정...
    args = parser.parse_args()
    
    # ROS2 노드 초기화
    rclpy.init()
    node = Node('kitti2bag')
    
    # rosbag 저장 위치 설정
    bag_path = args.output_dir
    os.makedirs(bag_path, exist_ok=True)
    
    # ROS2 bag 생성 및 설정
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3')
    
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    
    # 데이터 처리 및 저장...
    
    # 정리
    writer.close()
    node.destroy_node()
    rclpy.shutdown()
    
    print("KITTI-to-ROS2 bag conversion complete!")

if __name__ == "__main__":
    main() 