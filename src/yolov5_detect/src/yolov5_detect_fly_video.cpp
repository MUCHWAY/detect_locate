#include <iostream>
#include <chrono>
#include <cmath>

#include "yolov5_detect/cuda_utils.h"
#include "yolov5_detect/logging.h"
#include "yolov5_detect/common.hpp"
#include "yolov5_detect/utils.h"
#include "yolov5_detect/calibrator.h"
#include "yolov5_detect/preprocess.h"
#include "yolov5_detect/video.h"
#include "yolov5_detect/detect.h"
#include "yolov5_detect/sort.h"
#include "yolov5_detect/VideoCapture.hpp"


#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <unistd.h>
#include <thread>
#include <mutex>
#include <vector>
#include <ros/ros.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    string node_name;
    node_name = "yolov5_detect_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Publisher detect_pub = nh.advertise<yolov5_detect::detect>(node_name + "/detect", 10);

    string node_num;
    ros::param::get("~node_num", node_num);

    string engine_name;
    ros::param::get("~engine_name", engine_name);

    string video_name;
    ros::param::get("~video_name", video_name);

    vector<int> img_list;
    ros::param::get("~img_list",img_list);

    string video_out_path;
    ros::param::get("~video_out_path", video_out_path);

    int sort_p;
    ros::param::get("~sort_p", sort_p);

    Img_update img_update(video_name);
    thread img_update_thread(&Img_update::update, &img_update);//图像来自于相机

    cv::namedWindow("Display");
    cv::VideoWriter outputVideo;
    outputVideo.open(video_out_path + "detect.avi",  cv::VideoWriter::fourcc('M', 'P', '4', '2'), 20.0, cv::Size(1920, 1080));
    bool write = false;

    cv::Mat raw_img ;
    cv::Mat img;
    cv::Mat final;

    sleep(3);

    while(ros::ok()) 
    {
        auto start = chrono::system_clock::now();

        if(img_update.img_flag > 30) break;
        raw_img = img_update.return_img;
        img_update.img_flag ++;
        if(img_update.img_flag > 40) img_update.img_flag = 40;
        
        cv::resize(raw_img, final, cv::Size(960,540));
        cv::imshow("Display", final);
        int k = cv::waitKey(1); 
        static int num = 0;
        if(k == 115)  {
            write = true;
            cv::imwrite(video_out_path + to_string(num)+".jpg", raw_img);
            num ++;
        }
        if(write) cout<<write<<endl, outputVideo.write(raw_img);

        usleep(1000 * 20);

        auto end = chrono::system_clock::now();
        cout<<node_num + "_sum_time: "<<chrono::duration_cast<chrono::milliseconds>(end - start).count()<< "ms  "<<endl;
        cout<<"--------------------------------"<<endl;
    }

    cv::destroyWindow("Display");
    outputVideo.release();

    return 0;
}


