#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import serial
import os
import time
from grtk.msg import GNGGA

if __name__ == "__main__":
  rospy.init_node('GNGGA_pub',anonymous=True)

  ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)

  pub = rospy.Publisher("/GNGGA",GNGGA,queue_size=10)

  msg=GNGGA()

  while not rospy.is_shutdown():
    recv = ser.readline()
    data = recv.decode('utf8','ignore')
    data_h = data.find('GNGGA')

    if(data_h==1):
        # print(data)
        heading_str = data.split(',')
        heading = float( heading_str[1] )
        msg.GPS_heading = heading

        pub.publish(msg)
        
        print(heading)

   



   

