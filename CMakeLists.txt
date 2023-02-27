cmake_minimum_required(VERSION 3.22)
project(opencv_display_demo)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(ethz_apriltag2)
set(ETHZ_APRILTAG2_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/ethz_apriltag2/include)
add_subdirectory(aslam_cameras_april)
set(aslam_cameras_april_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/aslam_cameras_april/include)

find_library(MVS_LIB
  NAMES MvCameraControl
  PATHS "/opt/MVS/lib/64"
  )

include_directories(
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ETHZ_APRILTAG2_INCLUDES}
  ${aslam_cameras_april_INCLUDES}
  /opt/MVS/include
  include
)

add_executable(opencv_display_demo src/main.cpp)

target_link_libraries(opencv_display_demo ${OpenCV_LIBS} ${MVS_LIB} pthread aslam_cameras_april ethz_apriltag2)