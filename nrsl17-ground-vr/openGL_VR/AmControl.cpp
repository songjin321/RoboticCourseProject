#include "AmControl.h"
#include <math.h>
AmControl::AmControl()
{
}
bool AmControl::init_am_controler()
{
	if (!deltaCtrl.ConnectDevice())
	{
		std::cerr << "can not find GR-H series device!" << std::endl;
		return false;
	}
	return true;
}

bool AmControl::calculateMotorAngle(double theta_o, double px, double py)
{
	double theta1, theta2, theta3;
	double a, b, c, an1, an2, m, n, p;
	double c1, s1, c2, s2, k1;
	a = 17.9;    b = 11;    c = 1.3;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    

	an1 = c*cos(theta_o);
	an2 = c*sin(theta_o);
	n = px - an1;
	m = py - an2;
	p = n*n + m*m;

	c2 = (p - a*a - b*b) / (2 * a * b);
	k1 = 1.0 - c2*c2;
	s2 = -sqrt(k1);
	theta2 = atan2(s2, c2);

	s1 = ((a + b*c2)*m - b*s2*n) / p;
	c1 = ((a + b*c2)*n + b*s2*m) / p;
	theta1 = atan(s1 / c1);

	theta3 = theta_o - theta2 - theta1;

	motor_angle[0] = theta1 * 180 / 3.14 ;
	motor_angle[1] = theta2 * 180 / 3.14;
	motor_angle[2] = theta3 * 180 / 3.14;
	//std::cerr << "end angle = " << motor_angle[0] + motor_angle[1] + motor_angle[2] << std::endl;
	return true;
}

void AmControl::getMotorAngle(vector<double> &motor_angle_)
{
	double theta_o(0), px(0), py(0);
	bool blIO = false;
	blIO = deltaCtrl.SetFTGetPosStatus(dF, dT, nErrCode);
	deltaCtrl.GetPos(dPos);
	deltaCtrl.GetRad(dRad);
	deltaCtrl.GetState(statu);
	std::cerr << "Force feedback device end pose: \n x = " << dPos[0] << " y = " << dPos[1] << " z = " << dPos[2]  << std::endl;
	convertToEndEffect(theta_o, px, py);
	std::cerr << "Arm ideal end position:  \n x = " << px << " y = " << py << " theta_o = " << theta_o  << std::endl;
	calculateMotorAngle(theta_o, px, py);
	if (!isnan(motor_angle[0]) && !isnan(motor_angle[1]) && !isnan(motor_angle[2]))
	{
		// 第二个是逆时针转， 第三个0度时和第二个垂直，第二个电机动时第三个也会动，注意需要补偿。
		double angle_t[3];
		angle_t[0] = motor_angle[0];
		angle_t[1] = -motor_angle[1];
		angle_t[2] = (180 + motor_angle[2]- angle_t[1])*20/30 ;
		motor_angle_[0] = angle_t[0];
		motor_angle_[1] = angle_t[1];
		motor_angle_[2] = angle_t[2];
	}
}
void AmControl::convertToEndEffect(double &theta_o, double &px, double &py)
{
	// 将力反馈装置末端转化为相应的机械臂末端姿态
	double x_min = 0.085, x_max = 0.2;
	double y_min = -0.11, y_max = 0.11;
	double z_min = -0.077, z_max= 0.117;
	theta_o = (dPos[1] - y_min + 0.04)* 3.14/(y_max - y_min) - 1.57;
	px = (x_max - dPos[0]) * 30.2 / (x_max - x_min);
	py = (z_max+0.01 - dPos[2])* 30.2 / (z_max - z_min);// +0.01防止机械臂成水平干涉螺旋桨
}
AmControl::~AmControl()
{
	memset(dF, 0, sizeof(dF));
	memset(dT, 0, sizeof(dT));
	deltaCtrl.SetTorqueGetPosStatus(dT, nErrCode); // clear the force
	deltaCtrl.ReleaseDev();
}
