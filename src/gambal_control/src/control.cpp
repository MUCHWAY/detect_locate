#include "gambal_control/TcpSerial.hpp"
#include "gambal_control/CameraTracker.hpp"
#include "yolov5_detect/detect.h"
#include "time.h"
#include <ros/ros.h>
#include <iostream>
using namespace std;


TcpSerial  control{};
CameraTracker ct(control);
int update = 0;

void yoloCallback(const yolov5_detect::detect::ConstPtr& msg)
{
    if(msg->num.size()>0)
    {
        update = 0;
        ct.updateError(msg->box_x[0] -1024/2, 1024/2 - msg->box_y[0]);
    }
}

int main(int argc, char** argv){
    string node_name = "gambal_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/uav1/yolov5_detect_node/detect", 10, yoloCallback);

    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_PITCH, -90);
    usleep(1000 * 1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_FOLLOW_HEAD);
    usleep(1000 * 1000);
    
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_KEEP_POSITIVE);
    usleep(2000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_STOP);
    usleep(1000*100);
    
    ct.setStart(false);  //开始追踪

    while(ros::ok())
    {
        ros::spinOnce();    
        update ++;
        if(update > 100) update = 100;

        if(update > 10)
            ct.updateError(0,0,true);

        usleep(1000 * 200);
    }

    return 0; 
}