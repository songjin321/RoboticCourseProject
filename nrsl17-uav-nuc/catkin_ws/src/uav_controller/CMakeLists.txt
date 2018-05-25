cmake_minimum_required(VERSION 2.8.3)
project(uav_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  gazebo_msgs
  message_generation
  geometry_msgs
  nav_msgs
  mavros_msgs
)
catkin_package()
include_directories(include
${catkin_INCLUDE_DIRS}
)


add_executable(uav_controller src/uav_controller.cpp)
target_link_libraries(uav_controller ${catkin_LIBRARIES})


