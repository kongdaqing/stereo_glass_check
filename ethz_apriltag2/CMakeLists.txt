cmake_minimum_required(VERSION 3.0.2)
project(ethz_apriltag2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

