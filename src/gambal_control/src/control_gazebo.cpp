#include "gambal_control/TcpSerial.h"
#include "gambal_control/CameraTracker.h"
#include "gambal_control/detect_cb.h"
#include "gambal_control/camera_attitude.h"
#include <ros/ros.h>
#include <iostream>
using namespace std;


int main(int argc, char** argv){
    string node_name = "gambal_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    string gambal_IP;
    ros::param::get("~gambal_IP", gambal_IP);

    bool track;
    ros::param::get("~track", track);

    vector<int> img_size;
    ros::param::get("~img_size",img_size);

    TcpSerial  control(gambal_IP);
    CameraAttitudeReader reader;
    CameraTracker ct(control,reader);

    Detect_cb detect_cb(img_size);

    ros::Publisher attitude_pub = nh.advertise<gambal_control::camera_attitude>("/camera_attitude", 10);
    ros::Subscriber sub = nh.subscribe("yolov5_detect_node/detect", 10, &Detect_cb::yoloCallback, &detect_cb, ros::TransportHints().tcpNoDelay());

    control.pitch_follow(-90);
    control.focus();

    ct.setStart(track);  //开始追踪

    gambal_control::camera_attitude atti;

    while(ros::ok())
    {   
        cout<<"attitude: "<<reader.pitch<<" "<<reader.yaw<<endl;
        atti.pitch = reader.pitch;
        atti.yaw = reader.yaw_mag;

        attitude_pub.publish(atti);
        
        ros::spinOnce();    
        detect_cb.update ++;
        if(detect_cb.update > 100) detect_cb.update = 100;

        if(detect_cb.update > 10)
            ct.updateError(0,0,true);
        else {
            // cout<<detect_cb.error[0]<<' '<<detect_cb.error[1]<<endl;
            ct.updateError(detect_cb.error[0],detect_cb.error[1]);
        }
        
        usleep(1000 * 50);
    }

    return 0; 
}