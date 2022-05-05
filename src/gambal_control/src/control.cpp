#include "gambal_control/TcpSerial.h"
#include "gambal_control/CameraTracker.h"
#include "gambal_control/detect_cb.h"
#include <ros/ros.h>
#include <iostream>
using namespace std;

int main(int argc, char** argv){
    string node_name = "gambal_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    string gambal_IP;
    ros::param::get("~gambal_IP", gambal_IP);

    vector<int> img_size;
    ros::param::get("~img_size",img_size);

    TcpSerial  control(gambal_IP);
    CameraTracker ct(control);

    Detect_cb detect_cb(img_size);

    ros::Subscriber sub = nh.subscribe("/uav1/yolov5_detect_node/detect", 10, &Detect_cb::yoloCallback, &detect_cb, ros::TransportHints().tcpNoDelay());

    control.pitch_follow(-90);

    control.focus();

    ct.setStart(true);  //开始追踪

    while(ros::ok())
    {
        ros::spinOnce();    
        detect_cb.update ++;
        if(detect_cb.update > 100) detect_cb.update = 100;

        if(detect_cb.update > 10)
            ct.updateError(0,0,true);
        else {
            cout<<detect_cb.error[0]<<' '<<detect_cb.error[1]<<endl;
            ct.updateError(detect_cb.error[0],detect_cb.error[1]);
        }
        

        usleep(1000 * 200);
    }

    return 0; 
}