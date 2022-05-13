#ifndef TCP_SERIAL_HPP_
#define TCP_SERIAL_HPP_

#include <cstdint>
#include "gambal_control/ClientSocket.h"
#include "gambal_control/SocketException.h"
#include "time.h"

#define TCP_SERIAL_IP "192.168.42.109"

#define TCP_SERIAL_MSG_SIZE 24
#define CAN_MESSAGE_SIZE 8

#define S_ID_PLATFORM 0x0F
#define S_ID_PLATFORM_2 0x1F

#define CMD_PLATFORM_ZOOM_TO_POINT 0x01
#define CMD_PLATFORM_FOLLOW_HEAD 0x02
#define CMD_PLATFORM_SET_BACK 0x03
#define CMD_PLATFORM_SET_SPEED 0x04
#define CMD_PLATFORM_START_TRACE 0x05
#define CMD_PLATFORM_STOP_TRACE 0x06
#define CMD_PLATFORM_SET_DOWN 0x07

#define CMD_PLATFORM_SET_ANGLE 0x08
#define CMD_PLATFORM_SET_ANGLE_YAW 0x01
#define CMD_PLATFORM_SET_ANGLE_PITCH 0x02
#define CMD_PLATFORM_SET_ANGLE_ZOOM 0x03

#define CMD_PLATFORM_SET_ZOOM 0x12
#define CMD_PLATFORM_SET_ZOOM_KEEP_ZOOM_IN 0x01
#define CMD_PLATFORM_SET_ZOOM_KEEP_ZOOM_OUT 0x02
#define CMD_PLATFORM_SET_ZOOM_STOP 0x03
#define CMD_PLATFORM_SET_ZOOM_TO_1 0x04
#define CMD_PLATFORM_SET_ZOOM_IN_BY_2 0x05
#define CMD_PLATFORM_SET_ZOOM_OUT_BY_2 0x06

#define CMD_PLATFORM_SET_FOCUS 0x13
#define CMD_PLATFORM_SET_FOCUS_KEEP_POSITIVE 0x01
#define CMD_PLATFORM_SET_FOCUS_KEEP_NEGTIVE 0x02
#define CMD_PLATFORM_SET_FOCUS_STOP 0x03
#define CMD_PLATFORM_SET_FOCUS_AUTO 0x04

#define CMD_PLATFORM_FOCUS_TO_POINT 0x14

#define CMD_PLATFORM_CALIBERATE 0xF0

class TcpSerial
{

public:

    TcpSerial(std::string ip)
    {
        this->ip=ip;
        init();
    }
    
    virtual ~TcpSerial()
    {
        delete clientPtr;
    }

    bool sendCommand(uint8_t s_id, uint8_t cmd, double args1 = 0, double args2 = 0, double args3 = 0, double args4 = 0)
    {
        uint8_t buffer[TCP_SERIAL_MSG_SIZE];
        fillMessage(buffer,s_id,cmd,args1,args2,args3,args4);
        try
        {
            clientPtr->sendBuffer(buffer,TCP_SERIAL_MSG_SIZE);
            return true;
        }
        catch(SocketException& e)
        {
            std::cout << e.description() << std::endl;
        }
        return false;
    }

    void pitch_follow(const int& pitch) {
        this->sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_ANGLE, CMD_PLATFORM_SET_ANGLE_PITCH, pitch);//
        usleep(1000 * 1000);
        //this->sendCommand(S_ID_PLATFORM, CMD_PLATFORM_FOLLOW_HEAD);//跟随机头
        //usleep(1000 * 1000);
    }

    void focus(void) {
        this->sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_KEEP_POSITIVE);
        usleep(2000*1000);
        this->sendCommand(S_ID_PLATFORM, CMD_PLATFORM_SET_FOCUS, CMD_PLATFORM_SET_FOCUS_STOP);
        usleep(1000*100);
    }

private:

    std::string ip;
    ClientSocket* clientPtr;

    void init()
    {
        clientPtr = new ClientSocket(ip,2000);
    }

    uint16_t wCRC_Table[16] =
    {
        0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
        0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
    };

    uint16_t crc_fly16(uint8_t* pBuffer, uint16_t length) 
    {
        uint16_t len = length;
        uint16_t tmp;
        uint16_t crcTmp;
        crcTmp = 0xFFFF;
        while (length-- > 0) 
        {
            tmp = wCRC_Table[(pBuffer[len - length - 1] ^ crcTmp) & 15] ^ (crcTmp >> 4);
            crcTmp = wCRC_Table[((pBuffer[len - length - 1] >> 4) ^ tmp) & 15] ^ (tmp >> 4);
        }
        return (crcTmp);
    }

    void fillMessage(uint8_t* pBuffer, uint8_t s_id, uint8_t cmd, double args1 = 0, double args2 = 0, double args3 = 0, double args4 = 0)
    {
        uint8_t* head = pBuffer;
        *(pBuffer++) = 0xEB;
        *(pBuffer++) = 0x90;
        *(pBuffer++) = 0x0A;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x00;
        *(pBuffer++) = 0x40;
        *(pBuffer++) = 0x88;

        uint8_t* canBuff = pBuffer;
        for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
            *(pBuffer++) = 0;

        canBuff[0] = s_id;
        canBuff[1] = cmd;
        switch (s_id)
        {
        case S_ID_PLATFORM:
            switch (cmd)
            {
            case CMD_PLATFORM_ZOOM_TO_POINT:
                canBuff[2] = (uint8_t)((uint16_t)args1 & 0x00ff);
                canBuff[3] = (uint8_t)((uint16_t)args1 >> 8);
                canBuff[4] = (uint8_t)((uint16_t)args2 & 0x00ff);
                canBuff[5] = (uint8_t)((uint16_t)args2 >> 8);
                break;

            case CMD_PLATFORM_SET_SPEED:
                canBuff[2] = (uint8_t)((int16_t)(args1 * 100) & 0x00ff);
                canBuff[3] = (uint8_t)((int16_t)(args1 * 100) >> 8);
                canBuff[4] = (uint8_t)((int16_t)(args2 * 100) & 0x00ff);
                canBuff[5] = (uint8_t)((int16_t)(args2 * 100) >> 8);
                break;

            case CMD_PLATFORM_START_TRACE:
                canBuff[2] = (uint8_t)((uint16_t)(args1) & 0x00ff);
                canBuff[3] = (uint8_t)((uint16_t)(args1) >> 8);
                canBuff[4] = (uint8_t)((uint16_t)(args2) & 0x00ff);
                canBuff[5] = (uint8_t)((uint16_t)(args2) >> 8);
                canBuff[6] = (uint8_t)(((uint16_t)(args3) >> 4) & 0x00ff);
                canBuff[7] = (uint8_t)(((uint16_t)(args4) >> 4) & 0x00ff);
                break;

            case CMD_PLATFORM_SET_ANGLE:
                canBuff[2] = (uint8_t)args1;
                switch ((uint8_t)args1)
                {
                case CMD_PLATFORM_SET_ANGLE_YAW:
                case CMD_PLATFORM_SET_ANGLE_PITCH:
                    canBuff[3] = (uint8_t)((int16_t)(args2 * 10) & 0x00ff);
                    canBuff[4] = (uint8_t)((int16_t)(args2 * 10) >> 8);
                    break;
                case CMD_PLATFORM_SET_ANGLE_ZOOM:
                    canBuff[3] = (uint8_t)((uint16_t)(args2 * 100) & 0x00ff);
                    canBuff[4] = (uint8_t)((uint16_t)(args2 * 100) >> 8);
                    break;
                }
                break;
        
            case CMD_PLATFORM_FOCUS_TO_POINT:
                canBuff[2] = (uint8_t)((uint16_t)args1 & 0x00ff);
                canBuff[3] = (uint8_t)((uint16_t)args1 >> 8);
                canBuff[4] = (uint8_t)((uint16_t)args2 & 0x00ff);
                canBuff[5] = (uint8_t)((uint16_t)args2 >> 8);
                canBuff[6] = 25;
                canBuff[7] = 25;
                break;

            case CMD_PLATFORM_SET_FOCUS:
            case CMD_PLATFORM_SET_ZOOM:
                canBuff[2] = (uint8_t)args1;
                break;

            case CMD_PLATFORM_FOLLOW_HEAD:
            case CMD_PLATFORM_SET_BACK:
            case CMD_PLATFORM_CALIBERATE:
            case CMD_PLATFORM_SET_DOWN:
            case CMD_PLATFORM_STOP_TRACE:
                break;
            }
            break;

        case S_ID_PLATFORM_2:
            break;
        }

        uint16_t crc = crc_fly16(head, TCP_SERIAL_MSG_SIZE - 2);
        *(pBuffer++) = (uint8_t)(crc & 0x00ff);
        *(pBuffer++) = (uint8_t)(crc >> 8);
    }

};

#endif