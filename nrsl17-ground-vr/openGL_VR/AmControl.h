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
* ����������ʼ���������豸��������������װ�õ�ĩ��λ�ú���Ӧ�����˶�ѧ��ϵ�õ��ؽ�ת�ǡ�
*/
class AmControl
{
public:

	AmControl();
	/*
	* ��ʼ���������豸���
	*/
	bool init_am_controler();

	/*
	* ��õ����ת��
	*/
	void getMotorAngle(vector<double> &motor_angle);

	~AmControl();
private:
	/*
	* ��֪ĩ�˵�λ�ã����ݾ���Ļ�е����״���������˶�ѧ������ĽǶ�
	* @param theta_o ĩ�˵ĽǶ�,��λ����
	* @param px ĩ����x�����ϵ�ֵ����λ����
	* @param py ĩ����y�����ϵ�ֵ����λ����
	*/
	bool calculateMotorAngle(double theta_o, double px, double py);

	/*
	* ���������ֱ���ֵӳ�䵽ʵ�ʵ�ĩ��ִ������λ��
	* x, y���ϵ�ӳ��ֵ���Ϊ(17.9+11+1.3 = 30.2)cm����СΪ0. theta_o��ȡֵ��ΧΪ0~90.
	* ����������x�Ḻ��ӳ��Ϊ��е�۵�x������z������ӳ��Ϊy������y�����ֵӳ��Ϊtheta_o��90�ȡ�
	* @param theta_o ĩ�˵ĽǶ�,��λ����
	* @param px ĩ����x�����ϵ�ֵ����λ����
	* @param py ĩ����y�����ϵ�ֵ����λ����
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

