# This message is compatible with PCL::PointCloud2

# 메시지 헤더
std_msgs/Header header

# 시간 정보
float32 start_ring_time
float32 end_ring_time

# 클라우드 메타데이터
int32[] start_ring_index
int32[] end_ring_index

# 거리 및 강도 정보
float32[] point_col_ind
float32[] point_range

# IMU 및 오도메트리 상태 정보 (중복 필드 통합)
int64 imu_available
int64 odom_available
int64 odom_vio_available
int64 imu_roll_init
int64 imu_pitch_init
int64 imu_yaw_init
int64 odom_reset_id

# 초기 위치/자세 추측 (명칭 통일)
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw

# 클라우드 특성 데이터
float32[] cloud_curvature
int32[] cloud_sort_idx
float32[] cloud_neighbor_picked
float32[] cloud_label

# 이미지 투영 정보 
sensor_msgs/Image range_image

# 클라우드 데이터
sensor_msgs/PointCloud2 cloud_deskewed
sensor_msgs/PointCloud2 cloud_corner
sensor_msgs/PointCloud2 cloud_surface

# 키프레임 관련 클라우드
sensor_msgs/PointCloud2 key_frame_cloud
sensor_msgs/PointCloud2 key_frame_color
sensor_msgs/PointCloud2 key_frame_poses
sensor_msgs/PointCloud2 key_frame_map

# VIO 관련 정보
# int64 odomVIOAvailable  # 주석 처리된 필드

# VIO 초기 추측
float32 odom_x
float32 odom_y
float32 odom_z
float32 odom_roll
float32 odom_pitch
float32 odom_yaw

# 특징 클라우드
sensor_msgs/PointCloud2 corner_features
sensor_msgs/PointCloud2 surface_features

# 아래 중복 필드들은 제거
# int32 imu_available      # 중복
# int32 odom_available    # 중복
# float32 init_guess_x     # 중복
# float32 init_guess_y     # 중복
# float32 init_guess_z     # 중복
# float32 init_guess_roll  # 중복
# float32 init_guess_pitch # 중복
# float32 init_guess_yaw   # 중복
