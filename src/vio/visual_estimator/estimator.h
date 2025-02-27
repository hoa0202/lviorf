#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "feature_manager.h"
#include "integration_base.h"
#include "parameters.h"

#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


class Estimator : public rclcpp::Node
{
  public:
    Estimator();
    ~Estimator() = default;

    void setParameter();

    // interface
    void processIMU(const std::vector<std::pair<double, sensor_msgs::msg::Imu::SharedPtr>> &imu_msgs);
    void processImage(const std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &feature);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    // void visualInitialAlignWithDepth();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    int failureCount;

    // ROS2 퍼블리셔/서브스크라이버
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_margin_cloud;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    
    rclcpp::TimerBase::SharedPtr process_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 기존 멤버 변수들 유지
    std::mutex mtx_imu;
    std::mutex mtx_feature;
    
    std::queue<std::pair<double, sensor_msgs::msg::Imu::SharedPtr>> imu_buf;
    std::queue<std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>>> feature_buf;
    
    nav_msgs::msg::Path path;
};
