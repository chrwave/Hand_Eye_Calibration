# -*- coding: utf-8 -*-
"""
Created on Fri Apr  9 09:44:34 2021

@author: Andre
"""

import cv2
import time
import numpy as np
import sys

class DTUCAMERA:
# Start default camera
    def __init__(self,CamDevice=0):
        # CamDevice, -1 inherited, 0 webcam/primary, 1 secoundary, 2 third....
        # Whether the program is run in python 2 or not
        self.python_3 = (sys.version_info.major == 3)
        self.video = cv2.VideoCapture(CamDevice,cv2.CAP_DSHOW)
        self.video.get(3)
        self.video.get(4)
        print("Resolution of camera: {0} x {1} ". format(self.video.get(3), self.video.get(4)))
        self.windowsizewidth = 1920/2
        self.windowsizeheight = 1080/2
        
        self.AvgFrame = 0
        
    def GetCameraResolution(self):
        self.video.get(3)
        self.video.get(4)
        print("Resolution of camera from get: {0} x {1} ". format(self.video.get(3), self.video.get(4)))
        
        
    def SetCameraResolution(self, width, height):
        self.video.set(3,width)
        self.video.set(4,height)
        print("Resolution set to: {0} x {1} ". format(self.video.get(3), self.video.get(4)))

    def TakePictures(self, num_frames, autofocus=0):
        time.sleep(1)
        if(autofocus==1):
            self.video.set(cv2.CAP_PROP_AUTOFOCUS,0)
        else:
            self.video.set(cv2.CAP_PROP_AUTOFOCUS,1)
        # Number of frames to capture        
        print("Capturing {0} frames".format(num_frames))
        self.video.read(cv2.CV_32F)
        # Start time
        start = time.time()
        ImageFrames = []
        # Grab a few frames
        for i in range(0, num_frames) :
            ret, frame = self.video.read(cv2.CV_32F)
            ImageFrames.append(np.float32(frame))
            #print(i)
        
        self.AvgFrame = 0
        for i in range(0, num_frames):
            self.AvgFrame = (ImageFrames[i])+self.AvgFrame
        
        self.AvgFrame = self.AvgFrame/num_frames
        self.AvgFrame = np.uint8(self.AvgFrame)    
        # End time
        end = time.time()
        # Time elapsed
        seconds = end - start
        print ("Time taken : {0} seconds".format(seconds))
        # Calculate frames per second
        fps  = num_frames / seconds
        print("Estimated frames per second : {0}".format(fps))
        return self.AvgFrame
    
    def TakeSinglePicture(self, autofocus=0):
        if(autofocus==1):
            self.video.set(cv2.CAP_PROP_AUTOFOCUS,0)
        else:
            self.video.set(cv2.CAP_PROP_AUTOFOCUS,1)
        # Number of frames to capture        
        self.video.read(cv2.CV_32F)
        # Start time
        start = time.time()
        # Grab a few frames
        ret, frame = self.video.read(cv2.CV_32F)
        
        self.AvgFrame = np.float32(frame)
    
        self.AvgFrame = np.uint8(self.AvgFrame)    
        # End time
        end = time.time()
        # Time elapsed
        seconds = end - start
        #print ("Time taken : {0} seconds".format(seconds))
        # Calculate frames per second
        #fps  = num_frames / seconds
        #print("Estimated frames per second : {0}".format(fps))
        return self.AvgFrame
    
    def LiveFeedback(self,autofocus=0):
        while(self.video.isOpened()):
            if(autofocus==1):
                self.video.set(cv2.CAP_PROP_AUTOFOCUS,0)
            else:
                self.video.set(cv2.CAP_PROP_AUTOFOCUS,1)
            
            ret,img = self.video.read()
            cv2.namedWindow('input', cv2.WINDOW_NORMAL)
            # ------ OVERLAY (OPTIONAL) ------
            cv2.line(img,(0,np.int16(1080/2)),(1920,np.int16(1080/2)),(255, 255, 255),thickness=2) # OPTIONAL
            cv2.line(img,(np.int16(1920/2),0),(np.int16(1920/2),1080),(255, 255, 255),thickness=2) # OPTIONAL
            cv2.putText(img,'-x',(0,500), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA) # OPTIONAL
            cv2.putText(img,'+x',(1800,500), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA) # OPTIONAL
            cv2.putText(img,'-y',(850,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA) # OPTIONAL
            cv2.putText(img,'+y',(850,1020), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA) # OPTIONAL
            # ------ END OF OVERLAY ------
            
            
            cv2.imshow("input",img)
            cv2.resizeWindow('input', np.int16(self.windowsizewidth), np.int16(self.windowsizeheight))
            if cv2.waitKey(1) == 27:
                break
            if cv2.waitKey(1) == 13:
                break
            
        
    def ShowImage(self,Image,Time=0):
        cv2.namedWindow('output', cv2.WINDOW_NORMAL)
        cv2.imshow('output',Image)
        # Resize the Window
        cv2.resizeWindow('output', np.int16(self.windowsizewidth), np.int16(self.windowsizeheight))
        cv2.waitKey(Time)
    
        cv2.destroyAllWindows()
        # Release video
        
    def ShowImageLive(self,name,Image):
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name,Image)
        # Resize the Window
        cv2.resizeWindow(name, np.int16(self.windowsizewidth), np.int16(self.windowsizeheight))
        # Release video
        
    def CameraIntrinsics(self): #Logictech c922 camera
        self.fx = 1441.85
        self.fy = 1440.92
        self.cx = 957.952
        self.cy = 538.352
        self.cameramtx = np.identity(3)
        self.cameramtx[0,0] = self.fx
        self.cameramtx[1,1] = self.fy
        self.cameramtx[0,2] = self.cx
        self.cameramtx[1,2] = self.cy
        return self.cameramtx
    
    def SetCameraIntrinsics(self, fx, fy, cx, cy): #Can adjust the Camera Intrinsics
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.cameramtx = np.identity(3)
        self.cameramtx[0,0] = self.fx
        self.cameramtx[1,1] = self.fy
        self.cameramtx[0,2] = self.cx
        self.cameramtx[0,1] = self.cy
        return self.cameramtx
        
    def ClearCamera(self): #Release camera from opencv
        self.video.release()
    
   

