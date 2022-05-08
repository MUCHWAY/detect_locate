#ifndef CAMERA_ATTITUDE_READER_HPP_
#define CAMERA_ATTITUDE_READER_HPP_

#include <pthread.h>
#include <stdlib.h>
#include <fcntl.h>
#include "stdio.h"
#include "termios.h"
#include "unistd.h"
#include "limits.h"
#include <stdint.h>
#include "time.h"

#define CAN_USB_DEV "/dev/ttyUSB0"
#define CAN_MSG_LEN 27

class CameraAttitudeReader
{

public:

    double yaw=0,pitch=0,yaw_mag=0;

    CameraAttitudeReader(void (*callback)(double,double,double)=nullptr, std::string can_usb_dev=CAN_USB_DEV)
    {
        this->can_usb_dev=can_usb_dev;
        this->callback=callback;
        init();
    }

    ~CameraAttitudeReader()
    {
        isEnding=true;
    }

private:

    pthread_t clock_pthread;

    bool isEnding=false;
    std::string can_usb_dev;
    void(*callback)(double,double,double);

    int fd=0;
    uint8_t RxBuff[1024]={0};
    

    void init()
    {
        fd = open(can_usb_dev.c_str(), O_RDWR|O_NOCTTY);
        while( fd < 0 )
        {
            std::cout << "COM (" << can_usb_dev << ") Open Failed !" << std::endl;
            usleep(1000*1000);
            fd = open(can_usb_dev.c_str(), O_RDWR|O_NOCTTY);
        }

        struct termios opt;
        tcgetattr(fd, &opt);                                //获取终端控制属性
        cfsetispeed(& opt, B115200);                        //指定输入波特率(若不设置系统默认9600bps)
        cfsetospeed(& opt, B115200);                        //指定输出波特率(若不设置系统默认9600bps)
    
        opt.c_cflag &= ~ INPCK;                             //不启用输入奇偶检测
        opt.c_cflag |= (CLOCAL |  CREAD);                   //CLOCAL忽略 modem 控制线,CREAD打开接受者
        opt.c_lflag &= ~(ICANON | ECHO | ECHOE |  ISIG);    //ICANON启用标准模式;ECHO回显输入字符;ECHOE如果同时设置了 ICANON，字符 ERASE 擦除前一个输入字符，WERASE 擦除前一个词;ISIG当接受到字符 INTR, QUIT, SUSP, 或 DSUSP 时，产生相应的信号
        opt.c_oflag &= ~ OPOST;                             //OPOST启用具体实现自行定义的输出处理
        opt.c_oflag &= ~(ONLCR | OCRNL);                    //ONLCR将输出中的新行符映射为回车-换行,OCRNL将输出中的回车映射为新行符
        opt.c_iflag &= ~(ICRNL |  INLCR);                   //ICRNL将输入中的回车翻译为新行 (除非设置了 IGNCR),INLCR将输入中的 NL 翻译为 CR
        opt.c_iflag &= ~(IXON | IXOFF | IXANY);             //IXON启用输出的 XON/XOFF流控制,IXOFF启用输入的 XON/XOFF流控制,IXANY(不属于 POSIX.1；XSI) 允许任何字符来重新开始输出
        opt.c_cflag &= ~ CSIZE;                             //字符长度掩码,取值为 CS5, CS6, CS7, 或 CS8,加~就是无
        opt.c_cflag |=  CS8;                                //数据宽度是8bit
        opt.c_cflag &= ~ CSTOPB;                            //CSTOPB设置两个停止位，而不是一个,加~就是设置一个停止位
        opt.c_cflag &= ~ PARENB;                            //PARENB允许输出产生奇偶信息以及输入的奇偶校验,加~就是无校验
        opt.c_cc[VTIME] = 0;                                //等待数据时间(10秒的倍数),每个单位是0.1秒  若20就是2秒
        opt.c_cc[VMIN] = 0;                                 //最少可读数据,非规范模式读取时的最小字符数，设为0则为非阻塞，如果设为其它值则阻塞

        tcflush(fd, TCIOFLUSH);                             //刷串口清缓存
        tcsetattr(fd, TCSANOW, &opt);

        if(callback!=nullptr)
        {
            (*callback)(0,0,0);
        }
        pthread_create(&clock_pthread, NULL, &CameraAttitudeReader::runByThread, this);
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

    void getCameraAttitude(uint8_t* msg)
    {
        uint16_t crc=(double)(((uint16_t)msg[26])<<8|msg[25]);
        if(crc!=crc_fly16(msg,CAN_MSG_LEN-2))
        {
            return;
        }
        pitch=(double)((int16_t)(((uint16_t)msg[20])<<8|msg[19]))/100.0;
        yaw_mag=(double)((uint16_t)(((uint16_t)msg[22])<<8|msg[21]))/100.0;
        yaw=(double)((int16_t)(((uint16_t)msg[24])<<8|msg[23]))/100.0;

        if(callback!=nullptr)
        {
            (*callback)(pitch,yaw_mag,yaw);
        }
    }

    static void* runByThread(void* self) 
    {
        return static_cast<CameraAttitudeReader*>(self)->run();
    }

    void* run()
    {
        bool head1=false,head2=false;
        int msg_len=0;
        int rx=0;
        uint8_t msgBuff[CAN_MSG_LEN]={0};
        
        while(!isEnding)
        {
            if((rx=read(fd, RxBuff, 1024))>0)
            {
                for(int i=0;i<rx;i++)
                {
                    if(!head1)
                    {
                        if(RxBuff[i]==0xEB)
                        {
                            msgBuff[0]=0xEB;
                            head1=true;
                        }
                    }
                    else
                    {
                        if(!head2)
                        {
                            if(RxBuff[i]==0x90)
                            {
                                msgBuff[1]=0x90;
                                head2=true;
                                msg_len=2;
                            }
                            else
                            {
                                head1=false;
                            }
                        }
                        else
                        {
                            msgBuff[msg_len]=RxBuff[i];
                            msg_len+=1;
                            if(msg_len>=CAN_MSG_LEN)
                            {
                                msg_len=0;
                                head1=false;
                                head2=false;
                                getCameraAttitude(msgBuff);
                            }
                        }
                    }
                }
            }
        }
        return nullptr;
    }

};

#endif