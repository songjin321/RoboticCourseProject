#ifndef _COMMON_
#define _COMMON_
#include <stdint.h>
#include <memory>
#include "CRC.h"
enum ContorlMsg
{
    heart_beat = 0,
    uav_left = 10,
    uav_right = 11,
    uav_forward = 12,
    uav_backward = 13,
    uav_up = 14,
    uav_down = 15,
    vr_rotate_horizontal = 20,
    vr_rotate_vertically = 22,
    motor_1_angle = 30,
    motor_2_angle = 31,
    motor_3_angle = 32,
    uav_grab = 40
};
void encodePacket(unsigned char id, double value, char *buff);
bool decodePacket(char *buff, unsigned char &id, double &value);
#endif // !_COMMON_


