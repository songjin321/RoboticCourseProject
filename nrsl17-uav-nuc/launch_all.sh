#!/bin/bash

#先打开动捕系统，设定固定在飞机上的刚体名字为three，运行vrpn
:roslaunch  vrpn_client_ros   sample.launch

#启动vr_zed, gs_ros_convert, serial_ros_flx和uav_controller
roslaunch uav_controller uav_controller.launch

#飞机上电启动mavros
source ~/uav_nuc/catkin_mavros/devel/setup.bash
roslaunch mavros px4.launch
