#include "gambal_control/TcpSerial.hpp"
#include "time.h"


int main() {
    TcpSerial  control{};
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_PITCH, -90);
    // control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_YAW, 80);
    usleep(1000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_FOLLOW_HEAD);
    usleep(1000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_KEEP_POSITIVE);
    usleep(2000*1000);
    control.sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_STOP);
    usleep(1000*100);
    
    // while(1) {
    //     usleep(1000*100);
    // }
    return 0; 
}