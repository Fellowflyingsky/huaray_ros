cmake_minimum_required(VERSION 3.0.2)
project(huaray_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    geometry_msgs
    nav_msgs
    )

catkin_package()
find_package(OpenCV  REQUIRED)


set(huaray_dir ${PROJECT_SOURCE_DIR}/lib)
link_directories(${huaray_dir})

set(huaray_libs
libVideoRender.so
libMVSDK.so
)

message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
include_directories(
include
${OpenCV_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${catkin_INCLUDE_DIRS}
)



add_executable(huaray_ros src/huaray_ros.cpp
)
target_link_libraries(huaray_ros
${OpenCV_LIBS}
${huaray_libs}
${catkin_LIBRARIES}
)
