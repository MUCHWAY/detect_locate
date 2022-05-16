#include <iostream>
#include <chrono>
#include <cmath>

#include "yolov5_detect/video.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <unistd.h>
#include <thread>
#include <mutex>
#include <vector>
#include <ros/ros.h>
using namespace std;

using namespace std;
using namespace cv;


int main(int argc, char** argv) {
    string node_name;
    node_name = "yolov5_detect_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    string video_name;
    ros::param::get("~video_name", video_name);

    vector<int> img_list;
    ros::param::get("~img_list",img_list);

    string video_out_path;
    ros::param::get("~video_out_path", video_out_path);

    string img_save_path;
    ros::param::get("~img_save_path", img_save_path);

    string img_topic;
    ros::param::get("~img_topic", img_topic);


    Ros_image ros_img(img_topic);
    thread ros_img_thread(&Ros_image::img_update, &ros_img); //图像来自于话题

    // Img_update img_update(video_name);
    // thread img_update_thread(&Img_update::update, &img_update);//图像来自于相机
    
    cv::namedWindow("Display");
    cv::VideoWriter outputVideo;
    outputVideo.open(video_out_path + "detect.avi",  cv::VideoWriter::fourcc('M', 'P', '4', '2'), 10.0, cv::Size(1920, 1080));
    bool write = false;

    cv::Mat raw_img ;
    cv::Mat img;
    cv::Mat final;

    int indx=0;

    while(ros::ok()) 
    {
        auto start = chrono::system_clock::now();

        time_t timeReal;
        time(&timeReal);
        struct timeval tv_now;
        struct timezone tz_now;

        tm* t = gmtime(&timeReal); 
        gettimeofday(&tv_now,&tz_now);
        string now_time = to_string(t->tm_year+1900) +'_'+ 
                          to_string(t->tm_mon+1) +'_'+ 
                          to_string(t->tm_mday) +'_'+ 
                          to_string(t->tm_hour+8) +'_'+ 
                          to_string(t->tm_min) +'_'+ 
                          to_string(t->tm_sec) +'_'+ 
                          to_string(tv_now.tv_usec/1000);    
        cout<<now_time<<endl;

        // cout << "millisecond : \t" <<tv_now.tv_usec/1000 << endl; // 毫秒
        // cout << "millisecond : \t" << tv_now.tv_sec*1000 + tv.tv_usec/1000 << endl; // 毫秒
        // cout << "micronsecond : \t" << tv_now.tv_sec*1000000 +tv.tv_usec <<endl; // 微妙
        

        // if(img_update.img_flag==0) break;
        // raw_img=img_update.get_img();

        if(ros_img.update < 20) {
            raw_img = ros_img.img.clone();
            ros_img.update ++;
            if(ros_img.update > 30) ros_img.update = 30;
        }
        else {
            cout<<"Image not updated!!!"<<endl;
            usleep(1000 * 50);
            continue;
        }
        
        // cv::resize(raw_img, final, cv::Size(1920,1080));
        cv::imshow("Display", raw_img);
        int k = cv::waitKey(50); 

        if(k == 115)  {
            write = true;
            cv::imwrite(img_save_path + now_time + ".jpg", raw_img);
        }

        // if(write) cout<<write<<endl, outputVideo.write(img);
        if(write) {
            if(indx==40) {
                indx = 0;
                cout<<write<<endl;
                cv::imwrite(img_save_path + now_time +".jpg", raw_img);
            }
        }else indx = 0;

        indx ++;

        auto end = chrono::system_clock::now();
        cout<<"sum_time: "<<chrono::duration_cast<chrono::milliseconds>(end - start).count()<< "ms  "<<endl;
        cout<<"--------------------------------"<<endl;
    }
    cv::destroyWindow("Display");
    outputVideo.release();

    return 0;
}


