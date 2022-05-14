#include "gambal_control/detect_cb.h"
#include "gambal_control/camera_attitude.h"
#include <ros/ros.h>
#include <iostream>
#include "gambal_control/gimbal_control.h"
using namespace std;

vector<double> getDegLUT(double FOV, int length)
{
    vector<double> degLUT;
    double coef=2*tan(FOV/2*0.017453292519943)/length;
    for(int i=0;i<length;i++)
        degLUT.push_back(atan(coef*(length/2-i))*57.295779513082);
    return degLUT;
}

int main(int argc, char** argv){

    string node_name = "gambal_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double initial_pitch;
    ros::param::get("~initial_pitch", initial_pitch);

    string gambal_IP;
    ros::param::get("~gambal_IP", gambal_IP);

    bool track;
    ros::param::get("~track", track);

    vector<int> img_size;
    ros::param::get("~img_size",img_size);

    string gimbal_topic_orientation;
    ros::param::get("~gimbal_topic_orientation",gimbal_topic_orientation);

    string gimbal_topic_command;
    ros::param::get("~gimbal_topic_command",gimbal_topic_command);

    gimbal_control control(gimbal_topic_orientation,gimbal_topic_command);
    Eigen::Vector3d gimbal_att_sp(0.0,initial_pitch,0.0);
    Eigen::Vector3d gimbal_att;
    
    double FOV_h=30.0;
    vector<double> dx2deg=getDegLUT(FOV_h,img_size[0]);
    vector<double> dy2deg=getDegLUT(FOV_h/img_size[0]*img_size[1],img_size[1]);

    Detect_cb detect_cb(img_size);

    ros::Publisher attitude_pub = nh.advertise<gambal_control::camera_attitude>("/camera_attitude", 10);
    ros::Subscriber sub = nh.subscribe("yolov5_detect_node/detect", 10, &Detect_cb::yoloCallback, &detect_cb, ros::TransportHints().tcpNoDelay());

    control.send_mount_control_command(gimbal_att_sp);

    gambal_control::camera_attitude atti;

    while(ros::ok())
    {   
        gimbal_att=control.get_gimbal_att();

        cout<<"attitude: "<<gimbal_att[1]<<" "<<gimbal_att[2]<<endl;
        atti.pitch = gimbal_att[1];
        atti.yaw = gimbal_att[2];
    
        attitude_pub.publish(atti);
        
        ros::spinOnce();    
        if(track)
        {
            detect_cb.update ++;
            if(detect_cb.update > 100) detect_cb.update = 100;

            if(detect_cb.update > 10)
            {
                // do nothing
            }
            else 
            {
                int dx,dy;
                dx=(int)(img_size[1]/2-detect_cb.error[0]);
                if(dx<0)
                    dx=0;
                else if(dx>=img_size[0])
                    dx=img_size[0]-1;
                dy=(int)(img_size[1]/2-detect_cb.error[1]);
                if(dy<0)
                    dy=0;
                else if(dy>=img_size[1])
                    dy=img_size[1]-1;

                gimbal_att_sp[1]=atti.pitch+dy2deg[dy];
                gimbal_att_sp[2]=atti.yaw+dx2deg[dx];
                control.send_mount_control_command(gimbal_att_sp);
            }
        }
        
        usleep(1000 * 50);
    }

    return 0; 
}