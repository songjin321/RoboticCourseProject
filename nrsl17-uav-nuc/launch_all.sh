#!/bin/bash

#�ȴ򿪶���ϵͳ���趨�̶��ڷɻ��ϵĸ�������Ϊthree������vrpn
:roslaunch  vrpn_client_ros   sample.launch

#����vr_zed, gs_ros_convert, serial_ros_flx��uav_controller
roslaunch uav_controller uav_controller.launch

#�ɻ��ϵ�����mavros
source ~/uav_nuc/catkin_mavros/devel/setup.bash
roslaunch mavros px4.launch
