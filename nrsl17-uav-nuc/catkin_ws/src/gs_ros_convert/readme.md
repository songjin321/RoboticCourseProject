gs_ros_convert
====================

## 1.功能描述
接受地面站发来的信息，并转发到ros中

## 2.订阅话题
无

## 3.发布话题
| 话题名称 | 话题类型 | 说明 |
|------------|------------|---------|
|"/uav_twist_expected" | geometry_msgs::TwistStamped | 表示地面站传来的期望的飞机运动速度
|”/am_angle“ | geometry_msgs::Vector3 | 机械臂三个电机的转角
|”/is_grab“ | bool | 表示是否进行抓取

## 4.节点通信
- 与uav_controller节点通信
> 关系：本节点的下游节点<br> 
> 说明：它订阅本节点的"/uav_twist_expected"来控制飞机

- 与uav_controller节点通信
> 关系：本节点的下游节点<br> 
> 说明：它订阅本节点的”/am_angle“来控制机械臂运动

- 与uav_controller节点通信
> 关系：本节点的下游节点<br> 
> 说明：它订阅本节点的”/is_grab“来控制机械臂抓取
