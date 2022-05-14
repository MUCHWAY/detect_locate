#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import serial
import os
import time
from grtk.msg import GNGGA
from grtk.msg import GPHDT

if __name__ == "__main__":
  rospy.init_node('uav_rtk_pub',anonymous=True)

  ser = serial.Serial('/dev/rtk', 115200, timeout=0.2)

  pos_pub = rospy.Publisher("/uav_rtk", GNGGA, queue_size=10)
  heading_pub = rospy.Publisher("/uav_rtk_heading", GPHDT, queue_size=10)

  rtk_pos_msg=GNGGA()
  rtk_heading_msg=GPHDT()

  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():
    recv = ser.readline()
    data = recv.decode('utf8','ignore')
    # print(data)

    # data = "$GNGGA,025754.00,4004.74102107,N,11614.19532779,E,1,18,0.7,63.3224,M,-9.7848,M,, *58"

    try:
      data_h = data.find('GNGGA')
      if(data_h != -1):
          data_list = data.split(',')
          # print(data_list)
          rtk_pos_msg.utc = float(data_list[1])
          rtk_pos_msg.lat = float(data_list[2])/100
          rtk_pos_msg.lon = float(data_list[4])/100
          rtk_pos_msg.qual = float(data_list[6])
          rtk_pos_msg.sats = int(data_list[7])
          rtk_pos_msg.alt = float(data_list[9])
          pos_pub.publish(rtk_pos_msg)
          print([rtk_pos_msg.lat, rtk_pos_msg.lon, rtk_pos_msg.alt])
          print(rtk_pos_msg)

      data_h = data.find('GNHDT')
      if(data_h != -1):
          data_list = data.split(',')
          # print(data_list)
          rtk_heading_msg.heading = (float(data_list[1]) + 90) % 360

          heading_pub.publish(rtk_heading_msg)
          print(rtk_heading_msg)
      # rate.sleep()
    except:
      pass
    print("-----------------------")