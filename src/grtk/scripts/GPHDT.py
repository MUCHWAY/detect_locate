#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy

import serial
import os
import time
from grtk.msg import GPHDT

if __name__ == "__main__":
  rospy.init_node('GPHDT_pub',anonymous=True)

  ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)

  pub = rospy.Publisher("/GPHDT_heading",GPHDT,queue_size=10)

  msg = GPHDT()

  while not rospy.is_shutdown():
    recv = ser.readline()
    data = recv.decode('utf8','ignore')
    data_h = data.find('HDT')

    if(data_h==3):
        # print(data)
        heading_str = data.split(',')
        heading = float( heading_str[1] )
        msg.GPS_heading = heading

        pub.publish(msg)
        
        print(heading)

   



   

