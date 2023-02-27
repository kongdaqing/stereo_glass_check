cmake_minimum_required(VERSION 3.0.2)
project(aslam_cameras_april)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${ETHZ_APRILTAG2_INCLUDES})

add_library(${PROJECT_NAME}
  src/GridCalibrationTargetBase.cpp
  src/GridCalibrationTargetAprilgrid.cpp
  )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ethz_apriltag2)
