#include "UAV_Control.h"
#include <iostream>
#define	NUC_IP "192.168.1.211"
#define NUC_CONTROL_PORT "4098"
int main()
{
	// test UAV_control class
	UAV_Control uav_control;
	if (!uav_control.init_uav_control(NUC_IP, NUC_CONTROL_PORT))
	{
		std::cerr << "init uav control error" << std::endl;
		return -1;
	}
	while (1)
	{
		std::cerr << "output" << std::endl;
		uav_control.publish_msg(uav_forward, 10);
	}
		return 0;
}