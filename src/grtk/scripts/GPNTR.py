#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from grtk.msg import GPNTR
import rospy

import serial
import os
import time

if __name__ == "__main__":
  rospy.init_node('GPNTR_pub',anonymous=True)

  # ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)

  pub = rospy.Publisher("/GPNTR",GPNTR,queue_size=10)

  msg=GPNTR()

  while not rospy.is_shutdown():
    # recv = ser.readline()
    # data = recv.decode('utf8','ignore')
    data = '$GPNTR,083314.00,2,10.678,-0.172,+4.955,+9.457,0*41'
    data_h = data.find('GPNTR')

    if(data_h==1):
        d = data.split(',')
        msg.qual = int(d[2])
        msg.Dis = float(d[3])
        msg.N = float(d[4])
        msg.E = float(d[5])
        msg.U = float(d[6])
        pub.publish(msg)
        


   



   

