#ifndef CAMERA_TRACKER_HPP_
#define CAMERA_TRACKER_HPP_

#include "gambal_control/CameraAttitudeReader.hpp"
#include "gambal_control/TcpSerial.h"
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define CONTROL_INTERVAL 0.05
#define TIMEOUT 2

#define K_P 0.10
#define K_I 0.01
#define K_D 0.01

#define MAX_SPEED 20
#define MIN_ERROR 10


class CameraTracker
{

public:

    CameraTracker(TcpSerial& ts, CameraAttitudeReader& car):ts(ts),car(car)
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
        updateError(0,0,true);
        this->isStart=isStart;
    }

    void updateError(int dx,int dy,bool isReset=false)
    {
        this->dx=dx;
        this->dy=dy;
        counter=0;
        if(isReset)
        {
            x_sum=0;
            x_pre=dx;
            y_sum=0;
            y_pre=dy;
        }
    }

    void recoverCameraState()
    {
        usleep(1000 * 200);
        ts.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_PITCH, -90.0);
        while(abs(car.pitch+90)>2)
        {
            usleep(1000 * 100);
            std::cout<<abs(car.pitch+90)<<std::endl;
        }
        ts.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_FOLLOW_HEAD);//跟随机头
            usleep(1000 * 1000);
    }

private:

    TcpSerial& ts;
    CameraAttitudeReader& car;
    pthread_t clock_pthread;

    bool isEnding=false;
    bool isStart=false;
    bool isRecovered=true;

    int dx=0,dy=0;
    int counter=0;

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
        int max_timeout_counter=(int)(TIMEOUT/CONTROL_INTERVAL);
        long interval=(long)(CONTROL_INTERVAL*1000000);
        while(!isEnding)
        {
            while(isStart)
            {
                if(isRecovered)
                    isRecovered=false;

                if(counter<max_timeout_counter)
                {
                    counter+=1;
                }
                else
                {
                    updateError(0,0,true);
                }
                ts.sendCommand(S_ID_PLATFORM,CMD_PLATFORM_SET_SPEED,updateX(dx),updateY(dy));
                std::cout<<"tracking"<<std::endl;
                usleep(interval);
            }
            if(!isRecovered)
            {
                isRecovered=true;
                recoverCameraState();
            }
            usleep(1000);
        }
        return nullptr;
    }

};

#endif