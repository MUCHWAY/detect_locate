#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from serial_tool.msg import GPS_heading
import rospy

import serial
import os
import time

if __name__ == "__main__":
  rospy.init_node('GPS_heading_pub',anonymous=True)

  ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)

  pub = rospy.Publisher("/GPS_heading",GPS_heading,queue_size=10)

  msg=GPS_heading()

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

   



   

