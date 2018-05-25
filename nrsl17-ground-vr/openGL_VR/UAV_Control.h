#pragma once
#include <string>
#include "common.h"
#include <winsock2.h>
/*
* 这个类主要用来通过udp协议将地面站的控制信息发送到小电脑上
*/
class UAV_Control
{
public:
	UAV_Control();
	/*
	* 初始化
	*/
	bool init_uav_control(std::string ip_address, std::string server_port);
	bool publish_msg(ContorlMsg msg, double value = 0);
	bool set_uav_velocity_zero();
	~UAV_Control();
private:
	double uav_left_velocity = 0;
	double uav_right_velocity = 0;
	double uav_forward_velocity = 0;
	double uav_backward_velocity = 0;
	ContorlMsg Msg_id;
	int s;
	struct sockaddr_in si_dest;
};

