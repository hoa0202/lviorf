#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0


// mtx lock for two threads
std::mutex mtx_lidar;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// feature publisher for VINS estimator
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;



void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    double cur_img_time = rclcpp::Time(img_msg->header.stamp).seconds();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::msg::PointCloud feature_points;
        feature_points.header.stamp = img_msg->header.stamp;
        feature_points.header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points.points.push_back(p);
                    feature_points.channels.push_back(p_id * NUM_OF_CAM + i);
                    feature_points.channels.push_back(cur_pts[j].x);
                    feature_points.channels.push_back(cur_pts[j].y);
                    feature_points.channels.push_back(pts_velocity[j].x);
                    feature_points.channels.push_back(pts_velocity[j].y);
                }
            }
        }

        // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        sensor_msgs::msg::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points.points);
        feature_points.channels.push_back(depth_of_points);
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature->publish(feature_points);

        // publish features in image
        if (pub_match->get_subscription_count() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            sensor_msgs::msg::Image::SharedPtr msg = 
                cv_bridge::CvImage(img_msg->header, "bgr8", tmp_img).toImageMsg();
            pub_match->publish(msg);
        }
    }
}


void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    // 0. TF2로 변환 정보 가져오기
    static tf2_ros::Buffer tfBuffer(this->get_clock());
    static tf2_ros::TransformListener tfListener(tfBuffer);
    
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(
            "vins_world", "vins_body_ros", 
            rclcpp::Time(laser_msg->header.stamp), 
            rclcpp::Duration::from_seconds(0.01));
    } 
    catch (tf2::TransformException &ex) {
        // RCLCPP_ERROR(this->get_logger(), "lidar no tf: %s", ex.what());
        return;
    }

    // TF2의 변환을 Eigen으로 변환하여 사용
    Eigen::Isometry3d transform_eigen;
    tf2::fromMsg(transformStamped.transform, transform_eigen);
    
    // 이어서 기존 변환 작업 수행...
    
    // PCL 변환 작업은 동일
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // trans lidar point to ros standard frame
    tf::Transform ros_to_lidar = camera_to_ros_transform.inverse() * lidar_to_camera_transform.inverse();
    double roll, pitch, yaw;
    tf::Matrix3x3(ros_to_lidar.getRotation()).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transOffset = pcl::getTransformation(ros_to_lidar.getOrigin().getX(), ros_to_lidar.getOrigin().getY(), ros_to_lidar.getOrigin().getZ(), roll, pitch, yaw);
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        PointType p_trans = laser_cloud_offset->points[i];

        if (p_trans.x >= 0 && abs(p_trans.y / p_trans.x) <= 10 && abs(p_trans.z / p_trans.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // TODO: transform to IMU body frame
    // add by YJZ
/*     // 4. offset T_lidar -> ros standard frame  
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset; */

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transform_eigen);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.seconds();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode(const std::string& name) 
        : Node(name)
    {
        // 파라미터 선언 및 가져오기
        this->declare_parameter("config_file", "");
        this->declare_parameter("image_topic", "/camera/image_raw");
        this->declare_parameter("lidar_topic", "/velodyne_points");
        
        std::string config_file;
        this->get_parameter("config_file", config_file);
        this->get_parameter("image_topic", IMAGE_TOPIC);
        this->get_parameter("lidar_topic", LIDAR_TOPIC);
        
        // ROS2에서는 -> 연산자로 퍼블리셔와 서브스크라이버에 접근
        pub_feature = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "vins/feature/feature_tracker", 1000);
        
        pub_match = this->create_publisher<sensor_msgs::msg::Image>(
            "vins/feature/feature_img", 1000);
        
        pub_restart = this->create_publisher<std_msgs::msg::Bool>(
            "vins/feature/restart", 1000);
        
        // QoS 프로파일 사용
        sub_image = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, rclcpp::SensorDataQoS(),
            std::bind(&FeatureTrackerNode::img_callback, this, std::placeholders::_1));
            
        if (!LIDAR_TOPIC.empty()) {
            sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                LIDAR_TOPIC, rclcpp::SensorDataQoS(),
                std::bind(&FeatureTrackerNode::lidar_callback, this, std::placeholders::_1));
        }
        
        // 기존 코드 유지
        readIntrinsicParameter(config_file);
        
        RCLCPP_INFO(this->get_logger(), "Feature Tracker Node 초기화 완료");
    }
    
private:
    // 콜백 함수 정의
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        // 헤더 타임스탬프 처리
        double cur_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9;
        
        // 이미지 처리
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1") {
            sensor_msgs::msg::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        } else {
            ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        }
        
        // 특징점 추출 처리
        cv::Mat show_img = ptr->image;
        TicToc t_r;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)
                trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_time);
            else
            {
                if (EQUALIZE)
                {
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                }
                else
                    trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

            #if SHOW_UNDISTORTION
                trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
            #endif
        }

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                if (j != 1 || !STEREO_TRACK)
                    completed |= trackerData[j].updateID(i);
            if (!completed)
                break;
        }

        // 결과 발행 - ROS2 스타일
        sensor_msgs::msg::PointCloud feature_points;
        feature_points.header = img_msg->header;
        feature_points.header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points.points.push_back(p);
                    feature_points.channels.push_back(p_id * NUM_OF_CAM + i);
                    feature_points.channels.push_back(cur_pts[j].x);
                    feature_points.channels.push_back(cur_pts[j].y);
                    feature_points.channels.push_back(pts_velocity[j].x);
                    feature_points.channels.push_back(pts_velocity[j].y);
                }
            }
        }

        // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        sensor_msgs::msg::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points.points);
        feature_points.channels.push_back(depth_of_points);
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature->publish(feature_points);

        // publish features in image
        if (pub_match->get_subscription_count() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            sensor_msgs::msg::Image::SharedPtr msg = 
                cv_bridge::CvImage(img_msg->header, "bgr8", tmp_img).toImageMsg();
            pub_match->publish(msg);
        }
    }
    
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
    {
        // 라이다 타임스탬프 확인
        double lidar_time = lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9;
        
        // 현재 이미지에서 추출된 특징점 변환
        std::vector<geometry_msgs::msg::Point32> features_2d;
        
        // 특징점 데이터가 있는 경우만 처리
        if (!trackerData.cur_pts.empty()) {
            for (const auto& pt : trackerData.cur_pts) {
                geometry_msgs::msg::Point32 p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0;
                features_2d.push_back(p);
            }
            
            // 특징 추적기에 PCL 콜백 호출
            trackerData.pcl_callback(lidar_msg, features_2d);
        }
    }
    
    // 필요한 메서드들
    bool readIntrinsicParameter(const std::string& config_file)
    {
        // ... 기존 코드 유지
        return true;
    }

private:
    // ROS2 퍼블리셔 및 서브스크라이버 선언
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;
    
    // 필요한 변수들
    std::string IMAGE_TOPIC;
    std::string LIDAR_TOPIC;
    bool SHOW_TRACK = true;
    
    // feature_tracker 인스턴스
    FeatureTracker trackerData;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>("feature_tracker");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
