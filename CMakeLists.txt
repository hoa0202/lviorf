cmake_minimum_required(VERSION 3.8)
project(lviorf)

# 기본 C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션 설정
add_compile_options(-Wall -Wextra -Wpedantic)

# GTSAM 링크 관련 정책 설정
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# ament 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

# PCL 및 기타 라이브러리 찾기
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# GeographicLib 찾기 - pkg-config 사용 (이미 설치됨)
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GEOGRAPHIC QUIET geographiclib)
  if(GEOGRAPHIC_FOUND)
    # pkg-config로 찾은 경우 해당 변수 설정
    set(GeographicLib_INCLUDE_DIRS ${GEOGRAPHIC_INCLUDE_DIRS})
    set(GeographicLib_LIBRARIES ${GEOGRAPHIC_LIBRARIES})
    message(STATUS "GeographicLib 발견: ${GeographicLib_LIBRARIES}")
  else()
    # 직접 경로 지정 (Ubuntu 일반적인 경로)
    message(STATUS "pkg-config로 GeographicLib을 찾지 못했습니다. 직접 경로 설정")
    set(GeographicLib_INCLUDE_DIRS "/usr/include")
    set(GeographicLib_LIBRARIES "Geographic")
  endif()
else()
  # pkg-config가 없는 경우 직접 설정
  set(GeographicLib_INCLUDE_DIRS "/usr/include")
  set(GeographicLib_LIBRARIES "Geographic")
endif()

# GTSAM 찾기 - 상세 설정 추가
find_package(GTSAM REQUIRED)
if(NOT GTSAM_FOUND)
  message(FATAL_ERROR "GTSAM을 찾을 수 없습니다. 설치 여부를 확인하세요.")
endif()

# GTSAM 링크 디렉토리 명시적 추가
link_directories(${GTSAM_LIBRARY_DIRS})

# common_lib 라이브러리 추가
add_library(common_lib SHARED 
  lib/common_lib.cpp
)

# common_lib에 필요한 의존성 추가
target_include_directories(common_lib
  PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/lib
    ${GeographicLib_INCLUDE_DIRS}  # GeographicLib 포함 경로 추가
)

target_link_libraries(common_lib
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${GeographicLib_LIBRARIES}  # GeographicLib 링크
)

# 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# 실행 파일 목록
add_executable(${PROJECT_NAME}_imageProjection_node
  src/lio/imageProjection.cpp
)

add_executable(${PROJECT_NAME}_mapOptimization_node
  src/lio/mapOptmization.cpp  
)

# laserMapping 노드 제거 (원래 없는 파일)
# add_executable(${PROJECT_NAME}_laserMapping_node
#   src/lio/laserMapping.cpp  
# )

# 정의된 대상에 의존성 추가
ament_target_dependencies(${PROJECT_NAME}_imageProjection_node
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros pcl_conversions
  tf2_eigen tf2_geometry_msgs visualization_msgs cv_bridge
)

ament_target_dependencies(${PROJECT_NAME}_mapOptimization_node
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros pcl_conversions
  tf2_eigen tf2_geometry_msgs visualization_msgs 
)

# 헤더 디렉토리 지정
target_include_directories(${PROJECT_NAME}_imageProjection_node
  PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}  # GeographicLib 포함 경로 추가
    include
)

target_include_directories(${PROJECT_NAME}_mapOptimization_node
  PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}  # GeographicLib 포함 경로 추가
    include
)

# 링크 라이브러리 지정
target_link_libraries(${PROJECT_NAME}_imageProjection_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gtsam
  gtsam_unstable
  ${GeographicLib_LIBRARIES}  # GeographicLib 링크
  ${${PROJECT_NAME}_LIBRARIES}
  common_lib
)

target_link_libraries(${PROJECT_NAME}_mapOptimization_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gtsam
  gtsam_unstable
  ${GeographicLib_LIBRARIES}  # GeographicLib 링크
  ${${PROJECT_NAME}_LIBRARIES}
  common_lib
)

# 인터페이스 의존성 추가
rosidl_target_interfaces(${PROJECT_NAME}_imageProjection_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 설치 규칙
install(TARGETS
  ${PROJECT_NAME}_imageProjection_node
  ${PROJECT_NAME}_mapOptimization_node
  common_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ament 패키지 마무리
ament_package() 