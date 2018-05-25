cmake_minimum_required(VERSION 2.8.3)
project(am_controller)

# OpenMP
message(STATUS "Checking for OpenMP support")
find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP supported. Adding flags ${OpenMP_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_definitions(-DUSE_OPENMP)
else (OPENMP_FOUND)
  message(STATUS "OpenMP not supported")
endif (OPENMP_FOUND)

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
  genmsg
)


add_message_files(FILES servoset_msg.msg Mat_Tba.msg success_flag.msg)
add_service_files(FILES servoset_srv.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
${catkin_INCLUDE_DIRS}
)

add_executable(serial_ros_flx src/serial_ros_flx.cpp src/Kin.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(serial_ros_flx ${catkin_LIBRARIES})
add_dependencies(serial_ros_flx am_controller_generate_messages_cpp)

add_executable(servo_seter src/servo_seter.cpp)
target_link_libraries(servo_seter ${catkin_LIBRARIES})
add_dependencies(servo_seter am_controller_generate_messages_cpp)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
