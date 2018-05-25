#pragma once
#include <iostream>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Windows.h>
#include "GRH\DeltaControl.h"
/*
* 该类用来初始化力反馈设备，并根据力反馈装置的末端位置和相应的逆运动学关系得到关节转角。
*/
class AmControl
{
public:

	AmControl();
	/*
	* 初始化力反馈设备句柄
	*/
	bool init_am_controler();

	/*
	* 获得电机的转角
	*/
	void getMotorAngle(vector<double> &motor_angle);

	~AmControl();
private:
	/*
	* 已知末端的位置，根据具体的机械臂形状，利用逆运动学求解电机的角度
	* @param theta_o 末端的角度,单位弧度
	* @param px 末端在x方向上的值，单位厘米
	* @param py 末端在y方向上的值，单位厘米
	*/
	bool calculateMotorAngle(double theta_o, double px, double py);

	/*
	* 将力反馈手柄的值映射到实际的末端执行器的位姿
	* x, y轴上的映射值最大为(17.9+11+1.3 = 30.2)cm，最小为0. theta_o的取值范围为0~90.
	* 将力反馈的x轴负向映射为机械臂的x轴正向，z轴正向映射为y轴正向，y轴最大值映射为theta_o的90度。
	* @param theta_o 末端的角度,单位弧度
	* @param px 末端在x方向上的值，单位厘米
	* @param py 末端在y方向上的值，单位厘米
	*/
	void convertToEndEffect(double &theta_o, double &px, double &py);

	CDeltaUSBControl deltaCtrl;
	BYTE statu;
	double dF[3] = { 0 };
	double dPos[3] = { 0 };
	double dRad[6] = { 0 };
	double dT[3] = { 0 };
	int nErrCode = 0;
	vector<double> motor_angle = {0,0,0};
};

