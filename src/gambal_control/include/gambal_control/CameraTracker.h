#ifndef CAMERA_TRACKER_HPP_
#define CAMERA_TRACKER_HPP_

#include "gambal_control/TcpSerial.h"
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define CONTROL_INTERVAL 0.05

#define K_P 0.10
#define K_I 0.01
#define K_D 0.01

#define MAX_SPEED 20
#define MIN_ERROR 10

class CameraTracker
{

public:

    CameraTracker(TcpSerial& ts):ts(ts)
    {
        init();
    }

    ~CameraTracker()
    {
        isEnding=true;
        isStart=false;
    }

    void setStart(bool isStart)
    {
        if(isStart)
        {
            x_sum=0;
            x_pre=0;
            y_sum=0;
            y_pre=0;
            dx=0;
            dy=0;
        }
        this->isStart=isStart;
    }

    void updateError(int dx,int dy,bool isReset=false)
    {
        this->dx=dx;
        this->dy=dy;
        if(isReset)
        {
            x_sum=0;
            x_pre=dx;
            y_sum=0;
            y_pre=dy;
        }
    }

private:

    TcpSerial& ts;
    pthread_t clock_pthread;

    bool isEnding=false;
    bool isStart=false;

    int dx=0,dy=0;

    long x_sum=0;
    int x_pre=0;
    long y_sum=0;
    int y_pre=0;

    void init()
    {
        pthread_create(&clock_pthread, NULL, &CameraTracker::runByThread, this);
    }

    double updateX(int x_now)
    {

        if(x_now<MIN_ERROR && x_now>-MIN_ERROR)
        {
            x_sum=0;
            x_pre=x_now;
            return 0;
        }
        else
        {
            x_sum+=x_now;
            double result=K_P*(x_now)+K_I*CONTROL_INTERVAL*x_sum+K_D/CONTROL_INTERVAL*(x_now-x_pre);
            x_pre=x_now;
            if(result>MAX_SPEED || result<-MAX_SPEED)
            {
                result=result>0?MAX_SPEED:-MAX_SPEED;
            }
            return result;
        }
    }

    double updateY(int y_now)
    {
        if(y_now<MIN_ERROR && y_now>-MIN_ERROR)
        {
            y_sum=0;
            y_pre=y_now;
            return 0;
        }
        else
        {
            y_sum+=y_now;
            double result=K_P*(y_now)+K_I*CONTROL_INTERVAL*y_sum+K_D/CONTROL_INTERVAL*(y_now-y_pre);
            y_pre=y_now;
            if(result>MAX_SPEED || result<-MAX_SPEED)
            {
                result=result>0?MAX_SPEED:-MAX_SPEED;
            }
            return result;
        }
    }

    static void* runByThread(void* self) 
    {
        return static_cast<CameraTracker*>(self)->run();
    }

    void* run()
    {
        long interval=(long)(CONTROL_INTERVAL*1000000);
        clock_t tick=clock(),tock;
        
        while(!isEnding)
        {
            while(isStart)
            {
                ts.sendCommand(S_ID_PLATFORM,CMD_PLATFORM_SET_SPEED,updateX(dx),updateY(dy));
                usleep(interval);
            }
            usleep(1000);
        }
        return nullptr;
    }

};

#endif