#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import os
import time
from grtk.msg import GNGGA
import threading
import socket

import rospy
import socket
from nav_msgs.msg import *
import json
import math

IP='192.168.42.112'
PORT=2000
BUFSIZE=1024

def caculate(start, end):
    #start[0] = latitude, start[1] = longitude, start[2] = altitude
    C_EARTH = 6378137.0
    pos = [0.00, 0.00, 0.00]
    #通过经纬度计算位置偏差
    deltaLon   = (start[1] - end[1]) / 57.3
    deltaLat   = (start[0] - end[0]) / 57.3

    pos[0] = -1 * deltaLon * C_EARTH * math.cos(start[0]/57.3)
    pos[1] = -1 * deltaLat * C_EARTH
    pos[2] = -1 * (start[2] - end[2])
    return pos

class Client:
    def __init__(self,ip=IP,port=PORT):
        self.ip=ip
        self.port=port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.dest_addr = ('192.168.42.255', 6000)

        t = threading.Thread(target = self.receive)
        t.daemon = True
        t.start()
    
    def receive(self):
        msg=GNGGA()
        pub = rospy.Publisher("/car_rtk", GNGGA, queue_size=10)

        start = [32.0174843916, 118.5146050334, 18.1051]

        while True:
            try:
                self.client.connect((self.ip,self.port))
                print('connected')
                while True:
                    data=self.client.recv(BUFSIZE)
                    if len(data)==0:
                        print('close')
                        break
                    string=data.decode('utf-8')
                    # print(string)

                    data_list = string.split(',')

                    end = [ float(data_list[2])/100, float(data_list[4])/100, float(data_list[9])]
                    print(end)
                    #  car_pos = caculate(start, [float(data_list[2])/100, float(data_list[4])/100, float(data_list[9])])
                    car_pos = caculate(start, end)
      
                    data = json.dumps([{'x':car_pos[0],'y':car_pos[1]}]).encode('utf-8')
                    self.udp_socket.sendto(data, self.dest_addr)

                    print(data)

                    msg.utc = float(data_list[1])
                    msg.lat = float(data_list[2])/100
                    msg.lon = float(data_list[4])/100
                    msg.qual = float(data_list[6])
                    msg.sats = int(data_list[7])
                    msg.alt = float(data_list[9])

                    pub.publish(msg)
            
            except Exception as err:
                print(err)

            finally:
                self.client.close()

    def close(self):
        self.server.close()


if __name__ == "__main__":
    rospy.init_node('car_rtk_pub',anonymous=True)
    cl = Client()
    rospy.spin()

