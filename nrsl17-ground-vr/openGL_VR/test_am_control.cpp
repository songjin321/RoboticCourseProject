
#include "UAV_Control.h"
#include <iostream>
#include "AmControl.h"
#include <vector>
#define	NUC_IP "192.168.1.211"
#define NUC_CONTROL_PORT "4098"
int main()
{


	// Creat and init the am_control object
	AmControl am_control;
	if (!am_control.init_am_controler())
	{
		std::cerr << "init am control error" << std::endl;
		return -1;
	}
	std::cerr << "init am control OK" << std::endl;

	/*
	// test UAV_control class
	UAV_Control uav_control;
	if (!uav_control.init_uav_control(NUC_IP, NUC_CONTROL_PORT))
	{
		std::cerr << "init uav control error" << std::endl;
		return -1;
	}
	*/

	// publish arm angle to nuc
	std::vector<double> am_angle = {0,0,0};
	while (1)
	{
		am_control.getMotorAngle(am_angle);
		std::cerr << "actual_angle1 = " << am_angle[0] << " actual_angle2 =  " << am_angle[1] << " actual_angle3 = " << am_angle[2] << "\n" << std::endl;

		Sleep(100);
		//uav_control.publish_msg(motor_1_angle, am_angle[0]);
		//uav_control.publish_msg(motor_2_angle, am_angle[1]);
		//uav_control.publish_msg(motor_3_angle, am_angle[2]);
	}
}