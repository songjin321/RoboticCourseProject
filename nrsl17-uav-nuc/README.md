# uav-nuc
基于VR的无人机机械臂项目的无人机部分，运行在无人机上的nuc小电脑上。主要实现以下功能：
- 接受地面站的指令，控制飞机和相机的运动
- 接受地面站的指令，控制机械臂运动
- 将相机采集到的图像发送给地面站

## 包含的主要内容
- catkin_mavros：编译通过的mavros部分
- catkin_ws：我们自己的包的工作空间

  + am_controller 机械臂控制部分
  + uav_controller 控制飞机和相机的运动
  + gs_ros_convert 将地面站传来的各种信号转为相应的topic
  + vr_zed 向地面站发送zed捕捉到的图像
  + vrpn_client 接受动捕的信号

## 如何运行

1.飞机上电启动mavros
> source ~/uav_nuc/catkin_mavros/devel/setup.bash
> roslaunch mavros px4.launch

2.先打开动捕系统，设定固定在飞机上的刚体名字为three，运行vrpn
> roslaunch  vrpn_client_ros   sample.launch

3.启动vr_zed
> rosrun vr_zed vr_zed

4.启动gs_ros_convert
> rosrun gs_ros_convert gs_ros_convert

5.启动serial_ros_flx
> roslaunch am_controller am_controller.launch

6.启动uav_controller
> rosrun uav_controller uav_controller

## 坐标系关系

MAVROS采用标准的ENU坐标系，
初始创建刚体时会保证刚体的orientation角度为零，所以为了准确控制飞机姿态，我们要保证机头方向朝向正北，然后再创建刚体。

注意是否需要进行vrpn映射到mavros的转换,这个vrpn包会自动进行转换

设定安全区域：
vrpn：x轴朝西，y轴向上，z轴向南         mavros：x轴朝北(-z)，y轴朝东(-x)，z轴朝上(y)
右下角：
x：2.247
y: 0.7
z: 0.15

左下角：
x：-2.033
y：-0.2
z：1

