#ifndef DETECT_CB_HPP_
#define DETECT_CB_HPP_

#include "yolov5_detect/detect.h"
#include <vector>
using namespace std;

class Detect_cb
{
private:
    vector<int> img_size;

public:
    int update;
    vector<int> error;
    
    Detect_cb(vector<int>& img_size);
    ~Detect_cb();
    void yoloCallback(const yolov5_detect::detect::ConstPtr& msg);
};

Detect_cb::Detect_cb(vector<int>& img_size)
{
    this->update = 0;
    this->error = {0, 0};
    this->img_size = img_size;
}

Detect_cb::~Detect_cb()
{
}

void Detect_cb::yoloCallback(const yolov5_detect::detect::ConstPtr& msg)
{
    if(msg->num.size()>0)
    {
        this->update = 0;
        this->error[0] = msg->box_x[0] - this->img_size[0] / 2;
        this->error[1] = this->img_size[1] / 2 - msg->box_y[0];
    }
}

#endif