#include "loop_detection.h"
#include "rclcpp/rclcpp.hpp"

LoopDetection::LoopDetection() : Node("loop_detection")
{
    this->declare_parameter("voc_file", "brief_k10L6.bin");
    this->declare_parameter("min_loop_dist", 30);
    
    std::string voc_file;
    this->get_parameter("voc_file", voc_file);
    this->get_parameter("min_loop_dist", MIN_LOOP_DIST);
    
    // 사전 로드
    voc = new BriefVocabulary(voc_file);
    db = new BriefDatabase(voc, false);
    
    // 퍼블리셔 초기화
    pub_match_img = this->create_publisher<sensor_msgs::msg::Image>("loop_detection/match_image", 100);
    pub_match_points = this->create_publisher<sensor_msgs::msg::PointCloud>("loop_detection/match_points", 100);
    
    // 서브스크라이버 초기화
    sub_image = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 100,
        std::bind(&LoopDetection::imageCallback, this, std::placeholders::_1));
    
    sub_pose = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 100,
        std::bind(&LoopDetection::poseCallback, this, std::placeholders::_1));
}

void LoopDetection::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 이미지 처리 로직
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg, "bgr8");
    
    // 기존 로직 유지...
}

void LoopDetection::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 위치 업데이트 로직
    current_pose = *msg;
    
    // 기존 로직 유지...
}

// 기존 메서드들 유지...