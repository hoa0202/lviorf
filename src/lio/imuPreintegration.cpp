#include "utility.h"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public rclcpp::Node
{
private:
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    // ROS2 TF2 버퍼 및 리스너
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped lidar2Baselink;

    // 퍼블리셔/서브스크라이버
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    nav_msgs::msg::Path imuPath;

    // 변환 행렬
    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffine;

    // 동기화 및 제어 변수들
    std::mutex mtx;
    std::string odomTopic;
    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;

    // Eigen::Affine3f 변환 함수
    Eigen::Affine3f odom2affine(nav_msgs::msg::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        
        // tf2 쿼터니언에서 RPY로 변환
        tf2::Quaternion tf_quat;
        tf2::fromMsg(odom.pose.pose.orientation, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

public:
    TransformFusion() : Node("transform_fusion"), tf_broadcaster_(this)
    {
        // 파라미터 로드
        this->declare_parameter("lidar_frame", "base_link");
        this->declare_parameter("baselink_frame", "base_link");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_topic", "odometry/imu");

        lidarFrame = this->get_parameter("lidar_frame").as_string();
        baselinkFrame = this->get_parameter("baselink_frame").as_string();
        odometryFrame = this->get_parameter("odom_frame").as_string();
        mapFrame = this->get_parameter("map_frame").as_string();
        odomTopic = this->get_parameter("odom_topic").as_string();

        // TF2 설정
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // lidarFrame과 baselinkFrame이 다른 경우 변환 찾기
        if(lidarFrame != baselinkFrame) {
            try {
                lidar2Baselink = tf_buffer_->lookupTransform(lidarFrame, baselinkFrame, 
                                                           tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
                            lidarFrame.c_str(), baselinkFrame.c_str(), ex.what());
            }
        }

        // 구독자 초기화
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "lviorf/mapping/odometry", 5,
            std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1));
        
        subImuOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            odomTopic + "_incremental", 2000, 
            std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1));

        // 발행자 초기화
        pubImuOdometry = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic, 2000);
        pubImuPath = this->create_publisher<nav_msgs::msg::Path>("lviorf/imu/path", 1);

        // 경로 메시지 초기화
        imuPath.header.frame_id = odometryFrame;
    }

    // 콜백 함수: LiDAR 오도메트리 처리
    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        lidarOdomTime = rclcpp::Time(odomMsg->header.stamp).seconds();
        lidarOdomAffine = odom2affine(*odomMsg);
    }

    // 콜백 함수: IMU 오도메트리 처리
    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        imuOdomQueue.push_back(*odomMsg);
        
        // 최신 LiDAR 오도메트리 확인
        if (lidarOdomTime == -1)
            return;

        while (!imuOdomQueue.empty())
        {
            // 큐에서 가장 오래된 IMU 오도메트리 가져오기
            if (rclcpp::Time(imuOdomQueue.front().header.stamp).seconds() <= lidarOdomTime)
            {
                imuOdomQueue.pop_front();
                continue;
            }

            // 최신 IMU 오도메트리 변환
            nav_msgs::msg::Odometry odomCur = imuOdomQueue.front();
            imuOdomQueue.pop_front();

            // IMU 오도메트리에 LiDAR 오도메트리 결합
            imuOdomAffine = lidarOdomAffine * odom2affine(odomCur);

            // 최종 오도메트리 생성
            nav_msgs::msg::Odometry laserOdometry;
            laserOdometry.header.stamp = odomCur.header.stamp;
            laserOdometry.header.frame_id = odometryFrame;
            laserOdometry.child_frame_id = baselinkFrame;

            // 변환 행렬에서 위치 및 방향 설정
            tf2::Transform laser_tf;
            Eigen::Quaternionf q(imuOdomAffine.rotation());
            Eigen::Vector3f t(imuOdomAffine.translation());
            
            tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
            laser_tf.setOrigin(tf2::Vector3(t.x(), t.y(), t.z()));
            laser_tf.setRotation(tf_q);

            // 오도메트리 메시지 채우기
            geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf_q);
            
            laserOdometry.pose.pose.position.x = t.x();
            laserOdometry.pose.pose.position.y = t.y();
            laserOdometry.pose.pose.position.z = t.z();
            laserOdometry.pose.pose.orientation = odom_quat;
            
            pubImuOdometry->publish(laserOdometry);

            // Transform 브로드캐스트
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = odomCur.header.stamp;
            transformStamped.header.frame_id = odometryFrame;
            transformStamped.child_frame_id = baselinkFrame;
            transformStamped.transform.translation.x = t.x();
            transformStamped.transform.translation.y = t.y();
            transformStamped.transform.translation.z = t.z();
            transformStamped.transform.rotation = odom_quat;
            tf_broadcaster_.sendTransform(transformStamped);

            // IMU 경로 업데이트
            geometry_msgs::msg::PoseStamped imuPose;
            imuPose.header = laserOdometry.header;
            imuPose.pose = laserOdometry.pose.pose;
            imuPath.header.stamp = odomCur.header.stamp;
            imuPath.poses.push_back(imuPose);
            pubImuPath->publish(imuPath);
        }
    }
};

class IMUPreintegration : public rclcpp::Node
{
private:
    // 퍼블리셔
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubIMUOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubIMUPath;

    // 서브스크라이버
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // IMU 데이터 큐 및 경로
    std::mutex mtx;
    nav_msgs::msg::Path imuPath;
    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;

    // 다양한 변수들
    std::string odometryFrame;
    std::string imuTopic;
    std::string odomTopic;

    // 속도, 위치, 오리엔테이션 관련 변수들
    // GTSAM 관련 변수들
    // ...

public:
    IMUPreintegration() : Node("imu_preintegration"), tf_broadcaster_(this)
    {
        // 파라미터 로드
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("imu_topic", "/imu/data");
        this->declare_parameter("odom_topic", "odometry/imu");
        
        odometryFrame = this->get_parameter("odom_frame").as_string();
        imuTopic = this->get_parameter("imu_topic").as_string();
        odomTopic = this->get_parameter("odom_topic").as_string();

        // 퍼블리셔 초기화
        pubIMUOdometry = this->create_publisher<nav_msgs::msg::Odometry>(
            odomTopic + "_incremental", 2000);
        
        pubIMUPath = this->create_publisher<nav_msgs::msg::Path>(
            "lviorf/imu/path_incremental", 1);

        // 서브스크라이버 초기화
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, 2000,
            std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1));
        
        subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/mapping", 5,
            std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1));

        // 경로 초기화
        imuPath.header.frame_id = odometryFrame;

        // 다른 초기화 로직들...
    }

    // 여기에 다른 멤버 함수들 추가...

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        // IMU 데이터 처리 로직
        std::lock_guard<std::mutex> lock(mtx);
        imuQueOpt.push_back(*imuMsg);
        imuQueImu.push_back(*imuMsg);
        
        // 실제 IMU 데이터 처리 로직은 여기에...
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        // 오도메트리 데이터 처리 로직
        // 여기에 구현...
    }

    // 필요한 다른 함수들 모두 구현...
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 병렬 실행을 위한 멀티 스레드 실행기
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // 노드 생성
    auto tf_node = std::make_shared<TransformFusion>();
    auto imu_node = std::make_shared<IMUPreintegration>();
    
    // 실행기에 노드 추가
    executor.add_node(tf_node);
    executor.add_node(imu_node);
    
    // 스핀
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
