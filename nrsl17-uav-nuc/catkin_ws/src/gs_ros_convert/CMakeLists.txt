cmake_minimum_required(VERSION 2.8.3)
project(gs_ros_convert)
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
add_message_files(
   DIRECTORY msg
   FILES
   arm_angle_grab.msg
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
  )


include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(gs_ros_convert src/gs_ros_convert.cpp src/common.cpp)
target_link_libraries(gs_ros_convert ${catkin_LIBRARIES})
