#include "UAV_Control.h"
#include <stdio.h>
#include <winsock2.h>
#include <iostream>
#include <Ws2tcpip.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define BUFF_LEN 15
#define _WINSOCK_DEPRECATED_NO_WARNINGS
UAV_Control::UAV_Control()
{
}

bool UAV_Control::init_uav_control(std::string server_ip, std::string server_port)
{
	//Initialise winsock
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		std::cerr << "Failed. Error Code : " << WSAGetLastError();
		return false;
	}

	//create socket
	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
	{
		std::cerr << "socket() failed with error code : " << WSAGetLastError();
		return false;
	}

	//setup address structure
	ULONG* destAddr = new ULONG;
	inet_pton(AF_INET, server_ip.c_str(), destAddr);
	si_dest.sin_family = AF_INET;
	si_dest.sin_port = htons(stoi(server_port));
	si_dest.sin_addr.s_addr = *destAddr;
	return true;
}

 
bool UAV_Control::publish_msg(ContorlMsg msg, double value)
{
	char buf[BUFF_LEN]; 
	encodePacket(msg, value, buf);
	if (sendto(s, buf, BUFF_LEN, 0, (struct sockaddr*) &si_dest, sizeof(si_dest)) == SOCKET_ERROR )
	{
		std::cerr << "send data failed!" << std::endl;
		return false;
	}
	else
	{
		return true;
	}

}

bool UAV_Control::set_uav_velocity_zero()
{
	uav_left_velocity = 0;
	uav_right_velocity = 0;
	uav_forward_velocity = 0;
	uav_left_velocity = 0;
	return false;
}

UAV_Control::~UAV_Control()
{
	closesocket(s);
	WSACleanup();
}


