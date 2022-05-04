#include "gambal_control/TcpSerial.hpp"
#include "gambal_control/CameraTracker.hpp"
#include "time.h"
#include "ros/ros.h"
#include "yolov5_detect/detect.h"
#include <iostream>
using namespace std;

TcpSerial  control{};
CameraTracker ct(control);

int Width=4096,Height=2160;
// int index=0;

void yoloCallback(const yolov5_detect::detect& msg)
{
    // cout<<msg.box_x<<endl;
    cout<<"cb"<<endl;
    // if(msg->num.size()>0)
    // {
    //     ct.updateError(msg->box_x.at(index)-Width/2,Height/2-msg->box_y.at(index));
    // }
}

int main() {

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/yolov5_detect_node/detect", 10, yoloCallback);

    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_PITCH, -90);
    // control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_YAW, 80);
    usleep(1000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_FOLLOW_HEAD);
    usleep(1000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_KEEP_POSITIVE);
    usleep(2000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_STOP);
    usleep(1000*100);
    
    // ct.setStart(true);  //开始追踪

    // //ros::spin();
    // while(1)
    // {
    //     ct.updateError(0,0,true);
    //     usleep(2*1000000);      //每2s复位一次
    // }
    return 0; 
}