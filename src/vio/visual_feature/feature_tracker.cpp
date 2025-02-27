#include "feature_tracker.h"
#include "rclcpp/rclcpp.hpp"

int FeatureTracker::n_id = 0;

std::map<int, std::vector<cv::Point2f>> prev_un_pts_map;
std::map<int, std::vector<cv::Point2f>> cur_un_pts_map;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker(const rclcpp::NodeOptions& options)
    : Node("feature_tracker", options)
{
    this->declare_parameter("max_cnt", 150);
    this->declare_parameter("min_dist", 30);
    this->declare_parameter("freq", 10);
    this->declare_parameter("show_track", true);
    
    this->get_parameter("max_cnt", MAX_CNT);
    this->get_parameter("min_dist", MIN_DIST);
    this->get_parameter("freq", FREQ);
    this->get_parameter("show_track", SHOW_TRACK);
    
    pub_img = this->create_publisher<sensor_msgs::msg::Image>("feature_img", 1000);
    pub_features = this->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
    
    sub_img = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 100,
        std::bind(&FeatureTracker::img_callback, this, std::placeholders::_1));
    
    n_id = 0;
    first_image_flag = true;
    hasPrediction = false;
    use_lidar = false;
    lidar_cloud.reset(new pcl::PointCloud<PointType>());
}

void FeatureTracker::img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    double cur_time = rclcpp::Time(img_msg->header.stamp).seconds();
    if (first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_time;
        return;
    }
    
    // 이미지 처리 주파수 제한
    if (round(1.0 * pub_count / (cur_time - first_image_time)) <= FREQ)
    {
        pub_count++;
        
        cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        cv::Mat show_img = ptr->image;
        
        // 특징점 추적 로직
        cv::Mat gray;
        cv::cvtColor(show_img, gray, cv::COLOR_BGR2GRAY);
        
        std::vector<cv::Point2f> n_pts;
        if (cur_pts.size() > 0)
        {
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, cur_pts, status, err);
            
            // 특징점 필터링 로직...
        }
        
        // 새 특징점 검출 로직...
        
        prev_gray = gray.clone();
        prev_pts = cur_pts;
        
        // PointCloud 메시지 생성 및 발행
        sensor_msgs::msg::PointCloud feature_points;
        feature_points.header = img_msg->header;
        
        for (auto &pt : cur_pts)
        {
            geometry_msgs::msg::Point32 p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 1;
            
            feature_points.points.push_back(p);
        }
        
        pub_features->publish(feature_points);
        
        // 트래킹 결과 시각화
        if (SHOW_TRACK)
        {
            for (auto &pt : cur_pts)
                cv::circle(show_img, pt, 2, cv::Scalar(0, 255, 0), -1);
            
            sensor_msgs::msg::Image::SharedPtr track_img = 
                cv_bridge::CvImage(img_msg->header, "bgr8", show_img).toImageMsg();
            pub_img->publish(*track_img);
        }
    }
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        // RCLCPP_DEBUG(this->get_logger(), "CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        // RCLCPP_DEBUG(this->get_logger(), "temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        // RCLCPP_DEBUG(this->get_logger(), "set mask begins");
        TicToc t_m;
        setMask();
        // RCLCPP_DEBUG(this->get_logger(), "set mask costs %fms", t_m.toc());

        // RCLCPP_DEBUG(this->get_logger(), "detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                // cout << "mask is empty " << endl;
                RCLCPP_WARN(this->get_logger(), "mask is empty");
            if (mask.type() != CV_8UC1)
                // cout << "mask type wrong " << endl;
                RCLCPP_WARN(this->get_logger(), "mask type wrong");
            if (mask.size() != forw_img.size())
                // cout << "wrong size " << endl;
                RCLCPP_WARN(this->get_logger(), "wrong size");
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        // RCLCPP_DEBUG(this->get_logger(), "detect feature costs: %fms", t_t.toc());

        // RCLCPP_DEBUG(this->get_logger(), "add feature begins");
        TicToc t_a;
        addPoints();
        // RCLCPP_DEBUG(this->get_logger(), "selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;

    // 수정: prev_un_pts_map과 cur_un_pts_map 업데이트
    prev_un_pts_map = cur_un_pts_map;
    
    prevLeftPtsMap.clear();
    for (size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    // 이전 라이다 점들 저장
    // PUB_THIS_FRAME은 pubLidarFrame 결정을 위한 플래그
    PUB_THIS_FRAME = false;

    // 라이다 데이터가 있는 경우 처리
    if (use_lidar && PUB_THIS_FRAME) {
        // 현재 프레임이 발행되는 경우에만 라이다 데이터 처리
        
        // 라이다-카메라 정합 처리
        if (!lidar_cloud->empty() && !forw_pts.empty()) {
            std::vector<geometry_msgs::msg::Point32> curr_features;
            
            // 현재 특징점을 geometry_msgs::Point32로 변환
            for (const auto& pt : forw_pts) {
                geometry_msgs::msg::Point32 p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0;
                curr_features.push_back(p);
            }
            
            // 정합 수행
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches = 
                lidar_camera_projection(lidar_cloud, curr_features);
                
            // 결과 처리
            for (size_t i = 0; i < matches.size(); i++) {
                if (i < ids.size()) {
                    feature_3d_points[ids[i]] = matches[i].first;
                }
            }
        }
        
        // 데이터 처리 후 플래그 리셋
        use_lidar = false;
    }
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        // RCLCPP_DEBUG(this->get_logger(), "FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // RCLCPP_DEBUG(this->get_logger(), "FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // RCLCPP_DEBUG(this->get_logger(), "FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    // RCLCPP_INFO(this->get_logger(), "reading paramerter of camera %s", calib_file.c_str());
    
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

double getCurrentTime()
{
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    return tv.tv_sec + tv.tv_nsec * 1e-9;
}

// PCL 콜백 구현
void FeatureTracker::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg, 
                                 const std::vector<geometry_msgs::msg::Point32>& features_2d)
{
    // 클라우드 변환
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*lidar_msg, *cloud);
    
    // 타임스탬프 설정
    lidar_timestamp = lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9;
    
    // 특징점 2D 좌표 저장
    feature_2d_points = features_2d;
    
    // 라이다 포인트 클라우드 복사
    *lidar_cloud = *cloud;
    
    // 라이다 데이터 사용 플래그 설정
    use_lidar = true;
    
    // 3D-2D 정합 수행
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches = 
        lidar_camera_projection(cloud, features_2d);
    
    // 정합 결과 처리
    for (size_t i = 0; i < matches.size(); i++) {
        // 3D 특징점 저장 (ID를 키로 사용)
        if (i < ids.size()) {
            feature_3d_points[ids[i]] = matches[i].first;
        }
    }
}

// 라이다-카메라 정합 구현
bool FeatureTracker::inLidarFOV(const pcl::PointXYZI& p, const geometry_msgs::msg::Point32& cam_p)
{
    // 라이다 FOV 확인 (카메라 이미지 내에 투영될 수 있는지)
    float cam_x = cam_p.x;
    float cam_y = cam_p.y;
    
    // 간단한 투영 확인: 이미지 크기 내에 있는지 확인
    return (0 <= cam_x && cam_x < COL && 0 <= cam_y && cam_y < ROW);
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FeatureTracker::lidar_camera_projection(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
    const std::vector<geometry_msgs::msg::Point32>& features)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches;
    
    // 라이다 클라우드가 비어있거나 특징점이 없는 경우
    if (cloud->empty() || features.empty()) {
        return matches;
    }
    
    // 각 특징점에 대해 가장 가까운 라이다 포인트 검색
    for (const auto& feature : features) {
        float min_dist = 10.0; // 최대 거리 임계값
        int min_idx = -1;
        
        // 단순 선형 탐색 (k-d 트리로 최적화 가능)
        for (size_t i = 0; i < cloud->size(); i++) {
            const auto& pt = cloud->points[i];
            
            // 라이다 FOV 확인
            if (!inLidarFOV(pt, feature)) {
                continue;
            }
            
            // 2D 거리 계산 (특징점 좌표계에서)
            float dx = feature.x - pt.x;
            float dy = feature.y - pt.y;
            float dist = std::sqrt(dx*dx + dy*dy);
            
            // 최소 거리 업데이트
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = i;
            }
        }
        
        // 일치하는 포인트 찾은 경우
        if (min_idx >= 0) {
            const auto& pt = cloud->points[min_idx];
            Eigen::Vector3d pt3d(pt.x, pt.y, pt.z);
            Eigen::Vector3d pt2d(feature.x, feature.y, 0); // 2D 좌표
            
            matches.push_back(std::make_pair(pt3d, pt2d));
        }
    }
    
    return matches;
}
