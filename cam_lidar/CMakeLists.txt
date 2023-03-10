cmake_minimum_required(VERSION 2.8.1)
project(cam_lidar)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# opencv
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(cam_lidar src/cam_lidar.cpp)
target_link_libraries(cam_lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})