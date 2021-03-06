#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from numpy import save
import rospy
import math
import numpy as np
import threading
import argparse
import datetime
import socket
from time import sleep,time
import json

from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *

from yolov5_detect.msg import detect
from camera_locate.msg import DroneState

from cam_pos import *

class DelayedQueue:
    datas=[]
    delay=None
    last_data=None

    def __init__(self,delay:datetime.timedelta=datetime.timedelta()):
        self.datas=[]
        self.delay=delay

    def push(self,value):
        now=datetime.datetime.now()
        self.datas.append((now,value))
        while now - self.datas[0][0] > self.delay:
            self.datas.pop(0)

class Watchdog():
    def __init__(self,delay=0.5,callback=None):
        self.callback=callback
        self.delay=delay
        self.isFeed=False
        self.t=threading.Thread(target = self.wait)
        self.t.daemon = True
        self.t.start()

    def wait(self):
        while True:
            sleep(self.delay)
            if not self.isFeed:
                if self.callback:
                    self.callback()
            self.isFeed=False

    def feed(self):
        self.isFeed=True

class Detect_Grtk():
    def __init__(self,ID, camera_mtx, camera_dist, resolution):
        self.id = ID

        self.uav_attitude = [0.00,0.00,0.00]
        self.uav_pos = [0.00,0.00,0.00]

        self.timestamp=0
        self.camera_pitch=-45
        self.new_det = [0,0]
        self.camera_pose=[0,0,0,0,0,0]
        self.targets=[]
        self.cam_to_world = [0.00,0.00,0.00] 

        def clean():
            self.cam_to_world = [0.00, 0.00 ,0.00]
            self.new_det = [0,0]
            self.targets=[]

        self.watchdog = Watchdog(callback=clean)

        self.uav_pos_queue = DelayedQueue( delay=datetime.timedelta(seconds=0, milliseconds=200) )
        self.uav_attitude_queue = DelayedQueue( delay=datetime.timedelta(seconds=0, milliseconds=200) )

        self.cam_pos = Camera_pos( camera_mtx, camera_dist, resolution)

    def getJsonData(self):
        if len(self.targets)>0:
            return json.dumps({'timestamp':self.timestamp,'uav':int(self.id[3:]),'pose':self.camera_pose,\
                'targets':self.targets}).encode('utf-8')     
        else:
            return json.dumps({'timestamp':int(round(time() * 1000)),'uav':int(self.id[3:]),'pose':self.camera_pose,\
                'targets':[]}).encode('utf-8')

    def sub(self):
        rospy.Subscriber("yolov5_detect_node/detect", detect , self.yolov5_sub)
        rospy.Subscriber("prometheus/drone_state", DroneState , self.uav_attitude_sub)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped , self.local_pose_sub)

    def local_pose_sub(self,msg):
        # self.uav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.uav_pos_queue.push([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    def uav_attitude_sub(self,msg):
        # self.uav_attitude=[msg.attitude[0]*57.3, msg.attitude[1]*57.3, ((-1 * msg.attitude[2]*57.3) + 90 + 360) % 360]
        self.uav_attitude_queue.push([msg.attitude[0]*57.3, msg.attitude[1]*57.3, ((-1 * msg.attitude[2]*57.3) + 90 + 360) % 360])

    def yolov5_sub(self,data):
        if data.num:
            timestamp=int(round(time() * 1000))

            if self.uav_pos_queue.datas:
                self.uav_pos = self.uav_pos_queue.datas[0][1]

            if self.uav_attitude_queue.datas:
                self.uav_attitude = self.uav_attitude_queue.datas[0][1]

            p = [self.uav_pos[0], self.uav_pos[1], self.uav_pos[2], self.uav_attitude[2], self.uav_attitude[1], self.uav_attitude[0]]
            self.camera_pose=self.cam_pos.getCameraPose(p, self.camera_pitch)
            targets=[]

            for i in range(len(data.num)):
                d = []
                d.append(float(data.box_x[i]))
                d.append(float(data.box_y[i]))

                detect_x = data.box_x[i] - data.size_x[i]/2
                detect_y = data.box_y[i] - data.size_y[i]/2
                detect_w = data.size_x[i]
                detect_h = data.size_y[i]

                pix=self.cam_pos.pf.getFixedPix(detect_x,detect_y,detect_w,detect_h,self.camera_pose[4])     #??????????????????
                pix=self.cam_pos.point2point(pix)
                self.cam_to_world=self.cam_pos.pix2pos_2(self.camera_pose,self.cam_pos.newcameramtx,pix,inv_inmtx=self.cam_pos.inv_newcameramtx)  #??????????????????

                self.new_det=pix
                targets.append({'id':int(data.num[i]),'x':self.cam_to_world[0],'y':self.cam_to_world[1]})

            self.targets=targets
            self.timestamp=timestamp
            self.watchdog.feed()

    def save_send(self):
        print(self.id)
        print("{0:>8} {1:>6} {2:>6}".format("det_uv:",int(self.new_det[0]), int(self.new_det[1])))
        print("{0:>8} {1:>6} {2:>6} {3:>6}".format("uav_att:", round(self.uav_attitude[0], 2), round(self.uav_attitude[1], 2), round(self.uav_attitude[2], 2)))
        print("{0:>8} {1:>6} {2:>6} {3:>6}".format("uav_pos:",round(self.uav_pos[0],2),round(self.uav_pos[1],2),round(self.uav_pos[2],2)))
        print("{0:>8} {1:>6} {2:>6} {3:>6}".format("c_to_w:",round(self.cam_to_world[0],2),round(self.cam_to_world[1],2),round(self.cam_to_world[2],2)))
        print('------------------------------')
                    
if __name__ == "__main__":
    rospy.init_node("locate_node", anonymous=True)
    ID = rospy.get_param('~ID')

    calibrations_dict = rospy.get_param('~calibration')
    camera_mtx = calibrations_dict['camera_mtx']
    camera_dist = calibrations_dict['camera_dist']
    resolution = calibrations_dict['resolution']

    detect_grtk=Detect_Grtk(ID, camera_mtx, camera_dist, resolution)
    detect_grtk.sub()

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dest_addr_fusion = ('192.168.3.255', 5000)

    while not rospy.is_shutdown():
        detect_grtk.save_send()
        data=detect_grtk.getJsonData()
        udp_socket.sendto(data, dest_addr_fusion)
        sleep(0.2)
    udp_socket.close()

