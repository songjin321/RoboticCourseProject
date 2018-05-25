#pragma once
#include <winsock2.h>
#include <Windows.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <opencv2\opencv.hpp>
#include <string>
#pragma comment(lib, "Ws2_32.lib")
#include <mutex>
/*
* 用于接受小电脑上的zed相机发回的图像
*/
class NetCamera
{
public:
	NetCamera();
	bool initNetCamera(std::string address, std::string port);
	void grabImage(cv::Mat &left, cv::Mat &right);
	~NetCamera();
	cv::Mat zed_image[2];
	std::mutex mtx;
	bool run;
	bool new_frame;
private:
	SOCKET ConnectSocket;
};

