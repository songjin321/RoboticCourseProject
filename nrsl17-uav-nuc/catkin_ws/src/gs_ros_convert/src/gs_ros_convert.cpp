//
// Created by sj on 18-1-31.
// 仅仅只是做一个接受和发送的功能,udp socket
//
#include "common.h"
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TwistStamped.h>
#include "gs_ros_convert/arm_angle_grab.h"

#define BUFFLEN 15  //Max length of buffer
#define PORT 4098   //The port on which to send data
#define UAV_VELOCITY 0.01 

struct sockaddr_in si_me, si_other;
int s, slen=sizeof(si_me);
char buf[BUFFLEN];

bool initUdpSocket(std::uint16_t server_port)
{
    // creat a UDP socket
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        printf("socket init error");
        return false;
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(server_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    if( bind(s, (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        printf("bind error");
        return false;
    }
    // set udp recevice buff size
    int recvbuf_size = 500 * 15;
    setsockopt(s, SOL_SOCKET, SO_RCVBUF, &recvbuf_size, sizeof(recvbuf_size));
    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gs_ros_convert");
    ros::NodeHandle nh;
    if(!initUdpSocket(PORT))
    {
      	ROS_ERROR("initUdket error!"); 
	    return -1; 
    }
    ROS_INFO("initUdket success!\n");
    ros::Publisher uav_twist_expected_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/uav_twist_expected", 10);
    ros::Publisher arm_signal_pub = nh.advertise<gs_ros_convert::arm_angle_grab>
            ("/arm_control_signal", 10);
   
    // arm control signal 
    gs_ros_convert::arm_angle_grab arm_signal;
    
    // use uav_twist_expected express the twist which we expected uav arrive. for simplicity
    // we use the angular along y axis to express the angle of camera.
    geometry_msgs::TwistStamped uav_twist_expected;

    ros::Rate r(30);
    while(ros::ok())
    {
        // set select
        /*
        struct timeval tv;
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(s, &readfds);
        tv.tv_sec = 5;
        select(s+1, &readfds, NULL, NULL, &tv);
        //try to receive some data, this is a blocking call
        if(!FD_ISSET(s, &readfds))
            continue;
        */
        int arm_count = 0;
        for(int i = 10; i > 0; i--)
        {
        if (recvfrom(s, buf, BUFFLEN, 0, (struct sockaddr *) &si_other, (socklen_t *)&slen) == -1)
        {
            ROS_ERROR("recvfrom error\n");
        }
        unsigned char id;
        double value;
        if(!decodePacket(buf, id, value))
        {
            ROS_ERROR("reveive a damaged package!\n");
        }
       switch ((int)id)
        {
            case uav_left:
                uav_twist_expected.twist.linear.y = UAV_VELOCITY;
                break;
            case uav_right:
                uav_twist_expected.twist.linear.y = -UAV_VELOCITY;
                break;
	          case uav_forward:
                uav_twist_expected.twist.linear.x = UAV_VELOCITY;
                break;
            case uav_backward:
                uav_twist_expected.twist.linear.x = -UAV_VELOCITY;
                break;
             case uav_up:
                uav_twist_expected.twist.linear.z = UAV_VELOCITY;
                break;
             case uav_down:
                uav_twist_expected.twist.linear.z = -UAV_VELOCITY;
                break;   
            case vr_rotate_horizontal:
                uav_twist_expected.twist.angular.z = value;
                break;
            case vr_rotate_vertically:
                uav_twist_expected.twist.angular.y = value;
                break;
            case motor_1_angle:
                arm_signal.angle1 = value;
                break;
            case motor_2_angle:
                arm_signal.angle2 = value;
                break;
            case motor_3_angle:
                arm_signal.angle3 = value;
                break;
            case uav_grab:
                arm_signal.is_grab = true;
                arm_count = 200;
                break;
            default:
                ROS_ERROR("unrecognized id!\n");
        }
        arm_count--;
        }
        uav_twist_expected.header.stamp = ros::Time::now();    
        uav_twist_expected_pub.publish(uav_twist_expected);
        arm_signal_pub.publish(arm_signal);
        
        if (arm_count < 0)
          arm_signal.is_grab = false;
        uav_twist_expected.twist.linear.x = 0;
        uav_twist_expected.twist.linear.y = 0;
        uav_twist_expected.twist.linear.z = 0;
        
        r.sleep();
    }
    return 0;
}
