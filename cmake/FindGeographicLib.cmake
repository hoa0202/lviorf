# 모듈: FindGeographicLib
# GeographicLib 라이브러리를 찾기 위한 CMake 모듈
# GeographicLib_INCLUDE_DIRS - 헤더 파일 디렉토리
# GeographicLib_LIBRARIES - 링크할 라이브러리
# GeographicLib_FOUND - 라이브러리가 발견됐는지 여부

find_path(GeographicLib_INCLUDE_DIR
  NAMES GeographicLib/Config.h
  PATH_SUFFIXES include
  PATHS
  /usr
  /usr/local
)

find_library(GeographicLib_LIBRARY
  NAMES Geographic
  PATH_SUFFIXES lib lib64
  PATHS
  /usr
  /usr/local
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeographicLib
  DEFAULT_MSG
  GeographicLib_LIBRARY
  GeographicLib_INCLUDE_DIR
)

if(GeographicLib_FOUND)
  set(GeographicLib_LIBRARIES ${GeographicLib_LIBRARY})
  set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
endif()

mark_as_advanced(
  GeographicLib_INCLUDE_DIR
  GeographicLib_LIBRARY
) 