#include "utility.h"
#include "lviorf/msg/cloud_info.hpp"

// 포인트 타입 정의
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)(double, timestamp, timestamp)
)

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (uint32_t, t, t) (int, ring, ring)
 )

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;
// common_lib_ 변수는 이미 utility.h에 선언되어 있으므로 여기서 다시 선언하지 않음

class ImageProjection : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex odoVIOLock;

    // ROS2 구독자 및 발행자
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lviorf::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom, subVIOOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;
    std::deque<nav_msgs::msg::Odometry> odomVIOQueue;
    
    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;
    
    int imuPointerCur;
    double *imuTime = new double[queueLength];  // 주의: 이름 충돌 방지
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];
    
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;
    
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    
    int deskewFlag;
    bool odomDeskewFlag;
    bool odomVIODeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;
    
    lviorf::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

public:
    ImageProjection() : 
        ParamServer("imageProjection"),
        imuPointerCur(0),
        firstPointFlag(true),
        deskewFlag(0),
        odomDeskewFlag(false),
        odomVIODeskewFlag(false)
    {
        // 구독 생성
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, 2000, std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1));
        
        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            odomTopic+"_incremental", 2000, std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1));
        
        subVIOOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            "lviorf/vins/odometry/imu_propagate_ros", 2000, std::bind(&ImageProjection::odometryVIOHandler, this, std::placeholders::_1));
        
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic, 5, std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));
        
        // 발행자 생성
        pubExtractedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lviorf/lidar/deskew/cloud_deskewed", 1);
        
        pubLaserCloudInfo = this->create_publisher<lviorf::msg::CloudInfo>(
            "lviorf/deskew/cloud_info", 1);
        
        allocateMemory();
        resetParameters();
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        fullCloud->clear();
        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;
        odomVIODeskewFlag = false;
        
        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void odometryVIOHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock3(odoVIOLock);
        odomVIOQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;
        
        if (!deskewInfo())
            return;
        
        projectPointCloud();
        publishClouds();
        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        // 포인트 클라우드 캐시
        cloudQueue.push_back(*laserCloudMsg);
        
        if (cloudQueue.size() <= 2)
            return false;
        
        // 클라우드 변환
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        
        // 센서 타입에 따른 처리
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Velodyne 형식으로 변환
            pcl::fromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::MULRAN)
        {
            // Velodyne 형식으로 변환
            pcl::fromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        }
        else if (sensor == SensorType::ROBOSENSE) {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            pcl::fromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;
            
            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown sensor type: %d", int(sensor));
            rclcpp::shutdown();
        }
        
        // 타임스탬프 가져오기
        cloudHeader = currentCloudMsg.header;
        timeScanCur = toSec(cloudHeader.stamp);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        
        // dense 플래그 확인
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(this->get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }
        
        // ring 채널 확인
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            
            if (ringFlag == -1)
            {
                RCLCPP_ERROR(this->get_logger(), 
                    "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }
        
        // 포인트 시간 확인
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            
            if (deskewFlag == -1)
                RCLCPP_WARN(this->get_logger(), 
                    "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }
        
        return true;
    }

    bool deskewInfo()
    {
        // IMU 백업 확인
        std::lock_guard<std::mutex> lock1(imuLock);
        if (imuQueue.empty())
            return false;
            
        cloudHeader = currentCloudMsg.header;
        timeScanCur = toSec(cloudHeader.stamp);
        
        // 포인트 클라우드 타입 확인
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            // Velodyne 또는 Livox LiDAR
            laserCloudIn->clear();
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Ouster LiDAR
            tmpOusterCloudIn->clear();
            pcl::fromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::ROBOSENSE)
        {
            // Robosense LiDAR
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            pcl::fromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;
            
            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }
        else if (sensor == SensorType::MULRAN)
        {
            // Mulran LiDAR
            tmpMulranCloudIn->clear();
            pcl::fromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            
            std::vector<double> timestamps;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
                timestamps.push_back(tmpMulranCloudIn->points[i].t);
                
            double min_timestamp = *std::min_element(timestamps.begin(), timestamps.end());
            
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++) {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = (src.t - min_timestamp) * 1e-6;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown LiDAR sensor type.");
            rclcpp::shutdown();
        }
        
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        
        // 잡음 제거 확인
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(this->get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }
        
        // 링 채널 확인
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }
        
        // 포인트 시간 확인
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                RCLCPP_WARN(this->get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }
        
        // IMU/odom pre-integration
        while (!imuQueue.empty())
        {
            // 가장 오래된 IMU 데이터가 현재 클라우드보다 오래된 경우
            if (toSec(imuQueue.front().header.stamp) <= timeScanCur)
                imuDeskewInfo();
            else
                break;
        }
        
        odomDeskewInfo();
        
        return true;
    }

    void imuDeskewInfo()
    {
        // imuTime 변수 이름 충돌 수정
        double currentImuTime = imuQueue.front().header.stamp.sec + imuQueue.front().header.stamp.nanosec * 1e-9;
        
        if (imuPointerCur == 0) {
            imuTime[0] = currentImuTime;  // imuTime 배열 사용
            // ...
        }
        
        // imuTime 배열 대신 currentImuTime 사용
        double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
        
        // 배열 업데이트
        imuTime[imuPointerCur] = currentImuTime;
        // ...
    }

    void odomDeskewInfo()
    {
        // 정지 상태인 경우 odom 정보 skip
        if (odomQueue.empty())
            return;
            
        // 자세 변화가 작으면 디스큐 disable
        if (odomQueue.front().pose.covariance[0] < 1e-3)
            return;
            
        // 포인트 클라우드 시작 시간에 가까운 odom 찾기
        nav_msgs::msg::Odometry startOdomMsg;
        
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            
            if (toSec(startOdomMsg.header.stamp) >= timeScanCur)
                break;
        }
        
        // 포인트 클라우드 종료 시간에 가까운 odom 찾기
        nav_msgs::msg::Odometry endOdomMsg;
        
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];
            
            if (toSec(endOdomMsg.header.stamp) >= timeScanEnd)
                break;
        }
        
        // 동일한 시간이면 첫 번째 포인트를 투영할 때 추적 오류 발생
        if (toSec(startOdomMsg.header.stamp) == toSec(endOdomMsg.header.stamp))
            return;
            
        // 시작 시점과 종료 시점 사이의 이동 계산
        Eigen::Affine3f transBegin = pcl::getTransformation(
            startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z,
            0, 0, 0);
            
        Eigen::Affine3f transEnd = pcl::getTransformation(
            endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z,
            0, 0, 0);
            
        // IMU 움직임 추출
        odomIncreX = transEnd.translation().x() - transBegin.translation().x();
        odomIncreY = transEnd.translation().y() - transBegin.translation().y();
        odomIncreZ = transEnd.translation().z() - transBegin.translation().z();
        
        // 이동량이 없으면 디스큐 비활성화
        odomDeskewFlag = (fabs(odomIncreX) > 0.001 || fabs(odomIncreY) > 0.001 || fabs(odomIncreZ) > 0.001);
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;
        
        // 포인트 시간이 현재 스캔 전에 있는 경우 첫 번째 IMU 사용
        int imuPointerFront = 0;
        
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }
        
        // 포인트가 첫 번째 IMU 이전인 경우
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            // 선형 보간
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        // 대략적인 시간 비율로 위치 보간
        *posXCur = 0; *posYCur = 0; *posZCur = 0;
        
        // 스캔 중간에 포인트가 있다고 가정
        double ratio = relTime / (timeScanEnd - timeScanCur);
        
        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || toSec(cloudHeader.stamp) < 0)
            return *point;
            
        // 현재 포인트의 IMU 회전값 찾기
        double pointTime = timeScanCur + relTime;
        float rotXCur = 0, rotYCur = 0, rotZCur = 0;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
        
        // 포인트 회전 값 제거
        float posXCur = 0, posYCur = 0, posZCur = 0;
        if (odomDeskewFlag)
            findPosition(relTime, &posXCur, &posYCur, &posZCur);
            
        if (firstPointFlag)
        {
            // 처음 포인트 정의
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }
        
        // 왜곡된 포인트 변환
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;
        
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;
        
        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        
        // 점들 디스큐
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;
            
            float range = common_lib_->pointDistance(thisPoint);
            
            // 모션 보정
            if (deskewFlag == 1)
            {
                float relTime = laserCloudIn->points[i].time;
                thisPoint = deskewPoint(&thisPoint, relTime);
            }
            
            // out of range points
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;
                
            extractedCloud->push_back(thisPoint);
        }
        
        // 추출한 클라우드 발행
        if (pubExtractedCloud->get_subscription_count() != 0)
        {
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*extractedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubExtractedCloud->publish(laserCloudTemp);
        }
    }

    void publishClouds()
    {
        // 클라우드 정보 발행
        cloudInfo.header = cloudHeader;
        
        // 발행
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    common_lib_ = std::make_shared<CommonLib::common_lib>("mapping");  // 전역 객체 초기화
    
    auto node = std::make_shared<ImageProjection>();
    
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Image Projection Started.\033[0m");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
