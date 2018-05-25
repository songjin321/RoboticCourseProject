#pragma once
#include <string>
#include "common.h"
#include <winsock2.h>
/*
* �������Ҫ����ͨ��udpЭ�齫����վ�Ŀ�����Ϣ���͵�С������
*/
class UAV_Control
{
public:
	UAV_Control();
	/*
	* ��ʼ��
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

