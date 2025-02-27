#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#define PCL_NO_PRECOMPILE 

// <!-- lviorf_yjz_lucky_boy -->
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "../lib/common_lib.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

// <!-- lviorf_localization_yjz_lucky_boy -->
std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN};

class ParamServer : public rclcpp::Node
{
public:
    // 프레임 관련 파라미터
    std::string robot_id;
    
    // 토픽 이름
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;
    
    // 프레임 ID
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;
    
    // GPS 설정
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    
    // PCD 저장 설정
    bool savePCD;
    string savePCDDirectory;
    
    // 라이다 센서 설정
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;
    
    // IMU 관련 파라미터
    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    
    // 외부 캘리브레이션 매트릭스
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;
    
    // 복셀 필터 파라미터
    float mappingSurfLeafSize;
    float surroundingKeyframeMapLeafSize;
    float loopClosureICPSurfLeafSize;
    float z_tollerance; 
    float rotation_tollerance;
    
    // CPU 파라미터
    int numberOfCores;
    double mappingProcessInterval;
    
    // 주변 맵 관련 파라미터
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // 루프 클로저 관련 파라미터
    bool visualLoopClosureEnableFlag;
    bool loopClosureEnableFlag;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    
    // 글로벌 맵 시각화 파라미터
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(const std::string& node_name = "lviorf_node")
        : rclcpp::Node(node_name)
    {
        // 파라미터 선언 및 초기값 설정
        this->declare_parameter("robot_id", "roboat");
        this->declare_parameter("lviorf.pointCloudTopic", "points_raw");
        this->declare_parameter("lviorf.imuTopic", "imu_correct");
        this->declare_parameter("lviorf.odomTopic", "odometry/imu");
        this->declare_parameter("lviorf.gpsTopic", "odometry/gps");
        
        this->declare_parameter("lviorf.lidarFrame", "base_link");
        this->declare_parameter("lviorf.baselinkFrame", "base_link");
        this->declare_parameter("lviorf.odometryFrame", "odom");
        this->declare_parameter("lviorf.mapFrame", "map");
        
        this->declare_parameter("lviorf.useImuHeadingInitialization", false);
        this->declare_parameter("lviorf.useGpsElevation", false);
        this->declare_parameter("lviorf.gpsCovThreshold", 2.0);
        this->declare_parameter("lviorf.poseCovThreshold", 25.0);
        
        this->declare_parameter("lviorf.savePCD", false);
        this->declare_parameter("lviorf.savePCDDirectory", "/Downloads/LOAM/");
        
        this->declare_parameter("lviorf.sensor", "");
        
        this->declare_parameter("lviorf.N_SCAN", 16);
        this->declare_parameter("lviorf.Horizon_SCAN", 1800);
        this->declare_parameter("lviorf.downsampleRate", 1);
        this->declare_parameter("lviorf.point_filter_num", 3);
        this->declare_parameter("lviorf.lidarMinRange", 1.0);
        this->declare_parameter("lviorf.lidarMaxRange", 1000.0);
        
        this->declare_parameter("lviorf.imuType", 0);
        this->declare_parameter("lviorf.imuRate", 500.0);
        this->declare_parameter("lviorf.imuAccNoise", 0.01);
        this->declare_parameter("lviorf.imuGyrNoise", 0.001);
        this->declare_parameter("lviorf.imuAccBiasN", 0.0002);
        this->declare_parameter("lviorf.imuGyrBiasN", 0.00003);
        this->declare_parameter("lviorf.imuGravity", 9.80511);
        this->declare_parameter("lviorf.imuRPYWeight", 0.01);
        
        this->declare_parameter("lviorf.extrinsicRot", std::vector<double>());
        this->declare_parameter("lviorf.extrinsicRPY", std::vector<double>());
        this->declare_parameter("lviorf.extrinsicTrans", std::vector<double>());
        
        this->declare_parameter("lviorf.mappingSurfLeafSize", 0.2);
        this->declare_parameter("lviorf.surroundingKeyframeMapLeafSize", 0.2);
        this->declare_parameter("lviorf.z_tollerance", FLT_MAX);
        this->declare_parameter("lviorf.rotation_tollerance", FLT_MAX);
        
        this->declare_parameter("lviorf.numberOfCores", 2);
        this->declare_parameter("lviorf.mappingProcessInterval", 0.15);
        
        this->declare_parameter("lviorf.surroundingkeyframeAddingDistThreshold", 1.0);
        this->declare_parameter("lviorf.surroundingkeyframeAddingAngleThreshold", 0.2);
        this->declare_parameter("lviorf.surroundingKeyframeDensity", 1.0);
        this->declare_parameter("lviorf.loopClosureICPSurfLeafSize", 0.3);
        this->declare_parameter("lviorf.surroundingKeyframeSearchRadius", 50.0);
        
        this->declare_parameter("lviorf.visualLoopClosureEnableFlag", false);
        this->declare_parameter("lviorf.loopClosureEnableFlag", false);
        this->declare_parameter("lviorf.loopClosureFrequency", 1.0);
        this->declare_parameter("lviorf.surroundingKeyframeSize", 50);
        this->declare_parameter("lviorf.historyKeyframeSearchRadius", 10.0);
        this->declare_parameter("lviorf.historyKeyframeSearchTimeDiff", 30.0);
        this->declare_parameter("lviorf.historyKeyframeSearchNum", 25);
        this->declare_parameter("lviorf.historyKeyframeFitnessScore", 0.3);
        
        this->declare_parameter("lviorf.globalMapVisualizationSearchRadius", 1e3);
        this->declare_parameter("lviorf.globalMapVisualizationPoseDensity", 10.0);
        this->declare_parameter("lviorf.globalMapVisualizationLeafSize", 1.0);
        
        // 파라미터 값 가져오기
        this->get_parameter("robot_id", robot_id);
        this->get_parameter("lviorf.pointCloudTopic", pointCloudTopic);
        this->get_parameter("lviorf.imuTopic", imuTopic);
        this->get_parameter("lviorf.odomTopic", odomTopic);
        this->get_parameter("lviorf.gpsTopic", gpsTopic);
        
        this->get_parameter("lviorf.lidarFrame", lidarFrame);
        this->get_parameter("lviorf.baselinkFrame", baselinkFrame);
        this->get_parameter("lviorf.odometryFrame", odometryFrame);
        this->get_parameter("lviorf.mapFrame", mapFrame);
        
        this->get_parameter("lviorf.useImuHeadingInitialization", useImuHeadingInitialization);
        this->get_parameter("lviorf.useGpsElevation", useGpsElevation);
        this->get_parameter("lviorf.gpsCovThreshold", gpsCovThreshold);
        this->get_parameter("lviorf.poseCovThreshold", poseCovThreshold);
        
        this->get_parameter("lviorf.savePCD", savePCD);
        this->get_parameter("lviorf.savePCDDirectory", savePCDDirectory);
        
        std::string sensorStr;
        this->get_parameter("lviorf.sensor", sensorStr);
        
        if (sensorStr == "velodyne") {
            sensor = SensorType::VELODYNE;
        } else if (sensorStr == "ouster") {
            sensor = SensorType::OUSTER;
        } else if (sensorStr == "livox") {
            sensor = SensorType::LIVOX;
        } else if (sensorStr == "robosense") {
            sensor = SensorType::ROBOSENSE;
        } else if (sensorStr == "mulran") {
            sensor = SensorType::MULRAN;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): %s", sensorStr.c_str());
            rclcpp::shutdown();
        }
        
        this->get_parameter("lviorf.N_SCAN", N_SCAN);
        this->get_parameter("lviorf.Horizon_SCAN", Horizon_SCAN);
        this->get_parameter("lviorf.downsampleRate", downsampleRate);
        this->get_parameter("lviorf.point_filter_num", point_filter_num);
        this->get_parameter("lviorf.lidarMinRange", lidarMinRange);
        this->get_parameter("lviorf.lidarMaxRange", lidarMaxRange);
        
        this->get_parameter("lviorf.imuType", imuType);
        this->get_parameter("lviorf.imuRate", imuRate);
        this->get_parameter("lviorf.imuAccNoise", imuAccNoise);
        this->get_parameter("lviorf.imuGyrNoise", imuGyrNoise);
        this->get_parameter("lviorf.imuAccBiasN", imuAccBiasN);
        this->get_parameter("lviorf.imuGyrBiasN", imuGyrBiasN);
        this->get_parameter("lviorf.imuGravity", imuGravity);
        this->get_parameter("lviorf.imuRPYWeight", imuRPYWeight);
        
        this->get_parameter("lviorf.extrinsicRot", extRotV);
        this->get_parameter("lviorf.extrinsicRPY", extRPYV);
        this->get_parameter("lviorf.extrinsicTrans", extTransV);
        
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();
        
        this->get_parameter("lviorf.mappingSurfLeafSize", mappingSurfLeafSize);
        this->get_parameter("lviorf.surroundingKeyframeMapLeafSize", surroundingKeyframeMapLeafSize);
        this->get_parameter("lviorf.z_tollerance", z_tollerance);
        this->get_parameter("lviorf.rotation_tollerance", rotation_tollerance);
        
        this->get_parameter("lviorf.numberOfCores", numberOfCores);
        this->get_parameter("lviorf.mappingProcessInterval", mappingProcessInterval);
        
        this->get_parameter("lviorf.surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        this->get_parameter("lviorf.surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        this->get_parameter("lviorf.surroundingKeyframeDensity", surroundingKeyframeDensity);
        this->get_parameter("lviorf.loopClosureICPSurfLeafSize", loopClosureICPSurfLeafSize);
        this->get_parameter("lviorf.surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);
        
        this->get_parameter("lviorf.visualLoopClosureEnableFlag", visualLoopClosureEnableFlag);
        this->get_parameter("lviorf.loopClosureEnableFlag", loopClosureEnableFlag);
        this->get_parameter("lviorf.loopClosureFrequency", loopClosureFrequency);
        this->get_parameter("lviorf.surroundingKeyframeSize", surroundingKeyframeSize);
        this->get_parameter("lviorf.historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        this->get_parameter("lviorf.historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        this->get_parameter("lviorf.historyKeyframeSearchNum", historyKeyframeSearchNum);
        this->get_parameter("lviorf.historyKeyframeFitnessScore", historyKeyframeFitnessScore);
        
        this->get_parameter("lviorf.globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        this->get_parameter("lviorf.globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        this->get_parameter("lviorf.globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);
    }

    // 타임스탬프를 초로 변환
    double toSec(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    // IMU 메시지 변환기
    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        
        // linear acceleration 회전
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        
        // angular velocity 회전
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        
        if (imuType) {
            // orientation 회전
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();
            
            if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1) {
                RCLCPP_ERROR(this->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
                rclcpp::shutdown();
            }
        }
        
        return imu_out;
    }
};

// 클라우드 발행 함수
template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& thisPub, 
                                          const T& thisCloud, 
                                          const builtin_interfaces::msg::Time& thisStamp, 
                                          const std::string& thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
        
    return tempCloud;
}

// ROS 시간 가져오기
template<typename T>
double ROS_TIME(T msg)
{
    auto stamp = msg->header.stamp;
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

// IMU 각속도 처리
template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

// IMU 가속도 처리
template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

// IMU RPY 처리
template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    
    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

#endif