#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import math

class pixFixer:
    RATIO_LUT:list
    INDEX_LUT:list
    ANGLE_LUT:list

    def __init__(self,w=4.4,d=2.8,h=1.5,o=0.1,img_height=1024,FOV_v=60):
        self.RATIO_LUT=[]
        self.INDEX_LUT=[]
        self.ANGLE_LUT=[]

        for i in range(91):
            i*=0.017453292519943
            self.RATIO_LUT.append([])
            self.INDEX_LUT.append([])
            for j in range(90):
                j*=0.017453292519943
                l,r,t,b,c,h1,h2,hh=self.getPosition(w,d,h,o,i,j)

                self.RATIO_LUT[-1].append((t-c)/(t-b))
                self.INDEX_LUT[-1].append((t-b)/(r-l))
        for i in range(90):
            self.RATIO_LUT.append([])
            self.INDEX_LUT.append([])
            for j in range(90):
                self.RATIO_LUT[-1].append(1-self.RATIO_LUT[89-i][j])
                self.INDEX_LUT[-1].append(self.INDEX_LUT[89-i][j])

        coef=2*np.tan(FOV_v/2*0.017453292519943)/img_height
        for i in range(img_height):
            self.ANGLE_LUT.append(np.arctan(coef*(img_height/2-i))*57.295779513082)
    
    def getPosition(self,w,d,h,o,i,j):
        return [-d*np.sin(j),w*np.cos(j),(w*np.sin(j)+d*np.cos(j))*np.sin(i)+h*np.cos(i)+o*np.cos(i),o*np.cos(i),(w*np.sin(j)+d*np.cos(j))*np.sin(i)/2, \
                d*np.cos(j)*np.sin(i)+o*np.cos(i),w*np.sin(j)*np.sin(i)+o*np.cos(i),h*np.cos(i)]

    def getFixedPix(self,x,y,w,h,pitch):
        c=int(y+h/2)
        if c<0:
            c=0
        elif c>=len(self.ANGLE_LUT):
            c=len(self.ANGLE_LUT)-1
            
        pitch=-int(pitch+self.ANGLE_LUT[c])
        hw_ratio=h/w
        if pitch<=0:
            return [x+w/2,y+h]
        elif pitch>=180:
            return [x+w/2,y]
        else:
            for i in range(90):
                if hw_ratio<self.INDEX_LUT[pitch][i]:
                    return [x+w/2,y+self.RATIO_LUT[pitch][i]*h]
            return [x+w/2,y+self.RATIO_LUT[pitch][-1]*h]

class Camera_pos:
    def __init__(self, camera_mtx, camera_dist, resolution):
        # ????????????:
        self.mtx = np.array([[camera_mtx[0], 0.00000000000, camera_mtx[2]],
                            [0.000000000000, camera_mtx[1], camera_mtx[3]],
                            [0.000000000000, 0.00000000000, 1.0000000000]])
        self.dist = np.array([[camera_dist[0], camera_dist[1], camera_dist[2], camera_dist[3], camera_dist[4]]])
        self.FOV_v = 2 * np.arctan(self.mtx[1,2]/self.mtx[1,1])*57.3
        self.size = resolution

        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (resolution[0], resolution[1]), 1, (resolution[0], resolution[1]))  # ??????????????????
        self.img_x_center=self.newcameramtx[0][2]
        self.img_y_center=self.newcameramtx[1][2]

        self.inv_mtx=np.linalg.inv(self.mtx)
        self.inv_newcameramtx=np.linalg.inv(self.newcameramtx)
        
        self.camera_pos = [0.00 , 0.00 ,0.00]
        self.line_distance=0
        
        self.pf=pixFixer(img_height=self.size[1], FOV_v=self.FOV_v)

    def point2point(self,detect):
        a = []
        a.append(detect)
        b = []
        b.append(a)
        target = np.array(b)

        xy_undistorted = cv2.undistortPoints(target, self.mtx, self.dist, None, self.newcameramtx) 
        return xy_undistorted[0][0]
    
    def pos(self,h,detect):
        a = []
        a.append(detect)
        b = []
        b.append(a)
        target = np.array(b)
        xy_undistorted = cv2.undistortPoints(target, self.mtx, self.dist, None, self.newcameramtx) 

        self.camera_pos[0] = (h*( xy_undistorted[0][0][0] -self.img_x_center))/self.newcameramtx[0][0]
        self.camera_pos[1] = -1* (h*( xy_undistorted[0][0][1] -self.img_y_center))/self.newcameramtx[1][1]
        self.camera_pos[2] = h
        self.line_distance = math.sqrt(math.pow(math.sqrt( math.pow(h,2) + math.pow(self.camera_pos[1],2) ),2)+math.pow(self.camera_pos[0],2))

        return xy_undistorted[0][0]
    
    def getCameraPose(self,uav_pose,camera_pitch):

        x,y,height,yaw,pitch,roll=uav_pose

        yaw=yaw*0.017453292519943
        pitch=pitch*0.017453292519943
        roll=roll*0.017453292519943
        camera_pitch=camera_pitch*0.017453292519943

        rotation_mat=np.mat([[np.cos(-yaw),-np.sin(-yaw),0],[np.sin(-yaw),np.cos(-yaw),0],[0,0,1]])\
                    *np.mat([[1,0,0],[0,np.cos(-pitch),-np.sin(-pitch)],[0,np.sin(-pitch),np.cos(-pitch)]])\
                    *np.mat([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])\
                    *np.mat([[1,0,0],[0,np.cos(camera_pitch),-np.sin(camera_pitch)],[0,np.sin(camera_pitch),np.cos(camera_pitch)]])

        beta = np.arctan2(rotation_mat[2,1], np.sqrt((rotation_mat[0,1])**2 + (rotation_mat[1,1])**2))

        err = float(0.001)
        if beta >= np.pi/2-err and beta <= np.pi/2+err:
            beta = np.pi/2
            # alpha + gamma is fixed
            alpha = 0.0
            gamma = np.arctan2(rotation_mat[0,2], rotation_mat[0,0])
        elif beta >= -(np.pi/2)-err and beta <= -(np.pi/2)+err:
            beta = -np.pi/2
            # alpha - gamma is fixed
            alpha = 0.0
            gamma = np.arctan2(rotation_mat[0,2], rotation_mat[0,0])
        else:
            alpha = np.arctan2(-(rotation_mat[0,1])/(np.cos(beta)), (rotation_mat[1,1])/(np.cos(beta)))
            gamma = np.arctan2(-(rotation_mat[2,0])/(np.cos(beta)), (rotation_mat[2,2])/(np.cos(beta)))

        yaw=-alpha*57.295779513082
        pitch=beta*57.295779513082
        roll=gamma*57.295779513082

        return [x,y,height,yaw,pitch,roll]

    def pix2pos_2(self,camera_pose:list,inmtx,pix:list,d=None,inv_inmtx=None):

        x,y,height,yaw,pitch,roll=camera_pose

        yaw=yaw*0.017453292519943
        pitch=pitch*0.017453292519943
        roll=roll*0.017453292519943

        rotation_mat=np.mat([[np.cos(-yaw),-np.sin(-yaw),0],[np.sin(-yaw),np.cos(-yaw),0],[0,0,1]])\
                    *np.mat([[1,0,0],[0,np.cos(pitch),-np.sin(pitch)],[0,np.sin(pitch),np.cos(pitch)]])\
                    *np.mat([[np.cos(roll),0,np.sin(roll)],[0,1,0],[-np.sin(roll),0,np.cos(roll)]])\
                    *np.mat([[1,0,0],[0,0,1],[0,-1,0]])

        inv_rotation_mat=np.mat([[1,0,0],[0,0,-1],[0,1,0]])\
                        *np.mat([[np.cos(-roll),0,np.sin(-roll)],[0,1,0],[-np.sin(-roll),0,np.cos(-roll)]])\
                        *np.mat([[1,0,0],[0,np.cos(-pitch),-np.sin(-pitch)],[0,np.sin(-pitch),np.cos(-pitch)]])\
                        *np.mat([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])

        T=-inv_rotation_mat*np.mat([x,y,height]).T
        if not isinstance(inv_inmtx,np.matrix):
            inv_inmtx=np.linalg.inv(inmtx)

        if not d:
            d=height/np.sin(-pitch)

        center=rotation_mat*(inv_inmtx*d*np.mat([inmtx[0,2],inmtx[1,2],1]).T-T)
        z=center[2,0]

        posNorm=rotation_mat*(inv_inmtx*np.mat([pix[0],pix[1],1]).T-T)

        pos=[x+(posNorm[0,0]-x)/(height-posNorm[2,0])*(height-z), y+(posNorm[1,0]-y)/(height-posNorm[2,0])*(height-z), 0]
        return pos