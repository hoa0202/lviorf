#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/bool.hpp>

// PCL 라이브러리 헤더 추가
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// TF2 헤더 추가
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TicToc 클래스 정의
class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

// PCL 포인트 타입 정의
typedef pcl::PointXYZI PointType;

// 특징 추적기 클래스
class FeatureTracker
{
public:
    FeatureTracker();
    
    void readImage(const cv::Mat &_img, double _cur_time);
    void setMask();
    void rejectWithF();
    void undistortedPoints();
    
    // PCL 콜백 메서드 추가
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg, 
                     const std::vector<geometry_msgs::msg::Point32>& features_2d);

    // 라이다-카메라 정합 메서드 추가
    bool inLidarFOV(const pcl::PointXYZI& p, const geometry_msgs::msg::Point32& cam_p);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> lidar_camera_projection(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
        const std::vector<geometry_msgs::msg::Point32>& features);
    
    // 유틸리티 함수
    bool inBorder(const cv::Point2f &pt);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                 std::vector<cv::Point2f> &curPtLeft, 
                 std::vector<cv::Point2f> &curPtRight,
                 std::vector<int> &ids,
                 std::map<int, cv::Point2f> &prevPtMap);
                 
    void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
    void reduceVector(std::vector<int> &v, std::vector<uchar> status);
    
    std::vector<cv::Point2f> undistortedPts(std::vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    
    cv::Mat mask;
    cv::Mat prev_img, cur_img, forw_img;
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    std::vector<cv::Point2f> prev_un_pts, cur_un_pts;
    std::vector<int> ids;
    std::vector<int> track_cnt;
    
    // 맵 변수 추가
    std::map<int, cv::Point2f> prevLeftPtsMap;
    std::map<int, std::vector<cv::Point2f>> prev_un_pts_map;
    std::map<int, std::vector<cv::Point2f>> cur_un_pts_map;
    
    camodocal::CameraPtr m_camera;
    
    double cur_time;
    double prev_time;
    
    static int n_id;
    
    // 예측 변수
    bool hasPrediction;
    std::vector<cv::Point2f> predict_pts;
    
    // 경계 설정
    int ROW = 0;
    int COL = 0;
    int MAX_CNT = 150;
    int MIN_DIST = 30;
    bool EQUALIZE = true;
    bool PUB_THIS_FRAME = false;
    bool SHOW_TRACK = true;
    
    // 라이다 처리 변수
    bool use_lidar = false;
    double lidar_timestamp = 0.0;
    pcl::PointCloud<PointType>::Ptr lidar_cloud;
    std::vector<geometry_msgs::msg::Point32> feature_2d_points;
    std::map<int, Eigen::Vector3d> feature_3d_points;
};

class DepthRegister : public rclcpp::Node
{
public:
    DepthRegister(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("depth_register", options)
    {
        // ROS2 퍼블리셔 생성
        pub_depth_feature = this->create_publisher<sensor_msgs::msg::PointCloud>("vins/feature/depth_feature", 1000);
        pub_depth_image = this->create_publisher<sensor_msgs::msg::Image>("vins/feature/depth_image", 1000);
        pub_depth_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("vins/feature/depth_cloud", 1000);
        
        // tf2 리스너 설정
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    sensor_msgs::msg::ChannelFloat32 get_depth(const rclcpp::Time& stamp_cur, const cv::Mat& imageCur,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& depthCloud,
                                          const std::vector<cv::Point2f>& features);

    // ROS2 스타일로 변경된 함수
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg,
                     const std::vector<geometry_msgs::msg::Point32>& features_2d);  // 네임스페이스 수정

private:
    // ROS2 퍼블리셔
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_depth_feature;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_cloud;
    
    // tf2 리스너와 버퍼
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 변환된 변수
    geometry_msgs::msg::TransformStamped transform;
};
