cmake_minimum_required(VERSION 3.0.2)
project(projection)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(projection src/projection.cpp)
target_link_libraries(projection ${catkin_LIBRARIES})