# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 10:16:57 2021

@author: Andreas   & Christian
        (s160066)    (s160071)
"""

import socket
import time
import numpy as np
import sys
import pandas as pd
import cv2
import os
from datetime import datetime

##############################################
#Paramters that can be changed
directory = 'D:\Scripts\PythonScripts' #Should be changed to your directory
ImageAmount = 25                # Amount of images
FactorPictureScaling = 1        # Should be in the power of 2. (1, 2, 4, 8, 16)
CheckerboardSquareSize = 0.025  # Size of checkerboard square(s) 
Checkerboard = (9,6)            # Checkerboard dimensions
##############################################
os.chdir(directory)


####################################################
#Andreas & Christian Library
sys.path.append(directory+'\Library\CameraLib')
import camera2robotcalib as cam
import DenavitHartenberg
####################################################

sys.path.append(directory+'\Library\config')
sys.path.append(directory+'\Library\modules_AJT_CHR')

import robot
import robotconfig as rcfg

from src.ur.class_ur import UR

##############################################
#Camera initialization 
##############################################
cam = cam.DTUCAMERA(1)
cam.SetCameraResolution(1920,1080)
cam.GetCameraResolution()
##############################################


imgmeanframes = []

HOST1 = rcfg.HOST_IP
PORT1 = 30003              # The same port as used by the server

ur = UR(HOST1, PORT1)

d = datetime.now()
d = d.strftime('%d-%m-%Yz_%H-%M-%St')

Picturefolder = 'PicturesUR5ARM_' + d
if not os.path.exists(Picturefolder):
    os.makedirs(Picturefolder)

path = str(os.getcwd()) + '/' + Picturefolder

positionList = []

for iteration in range(ImageAmount):
    #position = [3,2,1,4,5,6]
    print("Hold Enter or press Esc to take picture...")
    cam.LiveFeedback()
    imgmeanframes.append(cam.TakePictures(6))
    print("Iteration: {0} out of {1}". format(iteration+1, ImageAmount))
    filename = 'PictureNr%s.png' % (iteration)
    # Saving the image
    cv2.imwrite(os.path.join(path,filename), imgmeanframes[iteration])
    
    jointposition = ur.get_joints_radv2()
    theta = np.array(jointposition[0:6])
    T06 = DenavitHartenberg.DenavitHartenbergUR(theta, 'UR5')
    rvecs,__ = cv2.Rodrigues(T06[0:3,0:3])
    tvecs = T06[0:3,3]
    print(tvecs)
    position = np.concatenate((tvecs.reshape(1,3), rvecs.reshape(1,3)),axis=1)
    positionList.append(np.array(position))
    #raw_input("Press Enter to continue...")
    time.sleep(0.2)
    
cv2.destroyAllWindows()
    
fileobject = open(path+"/robotposition.txt","a")
for i in range(len(positionList)):
    #Prints in format: x, y, z, rx, ry, rz
    fileobject.write(str(positionList[i].tolist())+ "\n")
    
fileobject.close()

cam.ClearCamera()



# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((Checkerboard[1]*Checkerboard[0],3), np.float32)
objp[:,:2] = np.mgrid[0:Checkerboard[0],0:Checkerboard[1]].T.reshape(-1,2)*CheckerboardSquareSize #25mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space 
imgpoints = [] # 2d points in image plane.
#images = glob.glob('D:/Scripts/PythonScripts/*.jpg') #skal fjernes
idx = 0
UsefulPositions = []
#UsefulPositions = pd.DataFrame()
for fname in imgmeanframes:
#for fname in images: #skal fjernes
    #img = cv2.imread(fname,1)
    img = fname
#    scale_percent = 15 # percent of original size
#    width = int(img.shape[1] * scale_percent / 100)
#    height = int(img.shape[0] * scale_percent / 100)
    width = int(img.shape[1] / FactorPictureScaling)
    height = int(img.shape[0] / FactorPictureScaling)
    dim = (width, height)  
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, Checkerboard,None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img2 = cv2.drawChessboardCorners(resized, Checkerboard, corners2,ret)
        cam.ShowImage(img2,Time=1500)
        UsefulPositions.append(positionList[idx])
        
    idx = idx + 1
        

cv2.destroyAllWindows()
rotationmatrix = []
Bhomogenematrix = []
B = []
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print("rvecs: ",rvecs)
print("Space")
print("tvecs:", tvecs)    


#Undistort the picture Method 1, Easy Method
img = imgmeanframes[0]

width = int(img.shape[1] / FactorPictureScaling)
height = int(img.shape[0] / FactorPictureScaling)
 
dim = (width, height)
     
# resize image  
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
 # Find the rotation and translation vectors.



# undistort
undstlist = []
idx = 0
#images = glob.glob('D:/Scripts/PythonScripts/*.jpg') #skal fjernes
for img2 in imgmeanframes:
#for img2 in images: #skal fjernes
    #img = cv2.imread(img2,1)# skal fjernes
    img = img2# skal fjernes
    width = int(img.shape[1] / FactorPictureScaling)
    height = int(img.shape[0] / FactorPictureScaling)
    
    dim = (width, height)     
    # resize image
    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    undstlist.append(dst)
    cam.ShowImage(dst)
    cv2.waitKey(0)
    filename = 'CalibratedResultNr%s.png' % (idx)
    cv2.imwrite(os.path.join(path,filename),dst)
    #cv2.imshow('imgshow',dst)      
    idx = idx+1 
    
cv2.destroyAllWindows()


mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print ("total pixel error: ", mean_error/len(objpoints))

cam.ClearCamera()

np.set_printoptions(precision=8,suppress=True)

robotrvecs = []
robottvecs = []
for i in range(0,len(UsefulPositions)):
    robotrvecs.append(UsefulPositions[i][0][3:6])
    temp1 = np.array(UsefulPositions[i][0][0:2].reshape(1,2))
    temp2 = np.array(UsefulPositions[i][0][2].reshape(1,1))
    temp1 = np.concatenate((temp1,temp2),axis=1).reshape(3,)
    robottvecs.append(temp1.copy())
    
homogenousmatrixplaceholder = np.identity(4)

rm1,tm1 = cv2.calibrateHandEye(robotrvecs,robottvecs,rvecs,tvecs,method=cv2.CALIB_HAND_EYE_PARK)

#print(np.linalg.inv(Rm))
homogenousmatrixplaceholder = np.identity(4)
homogenousmatrixplaceholder[0:3,0:3] = rm1
homogenousmatrixplaceholder[0:3,3] = tm1.ravel()
print("Park-Martin Calibration Matrix (1):")
print(homogenousmatrixplaceholder)
print("")

rm2,tm2 = cv2.calibrateHandEye(robotrvecs,robottvecs,rvecs,tvecs,method=cv2.CALIB_HAND_EYE_TSAI)

homogenousmatrixplaceholder = np.identity(4)
homogenousmatrixplaceholder[0:3,0:3] = rm2
homogenousmatrixplaceholder[0:3,3] = tm2.ravel()
print("Tsai Calibration Matrix (2):")
print(homogenousmatrixplaceholder)
print("")


rm3,tm3 = cv2.calibrateHandEye(robotrvecs,robottvecs,rvecs,tvecs,method=cv2.CALIB_HAND_EYE_HORAUD)

homogenousmatrixplaceholder = np.identity(4)
homogenousmatrixplaceholder[0:3,0:3] = rm3
homogenousmatrixplaceholder[0:3,3] = tm3.ravel()
print("Horaud Calibration Matrix (3):")
print(homogenousmatrixplaceholder)
print("")


rm4,tm4 = cv2.calibrateHandEye(robotrvecs,robottvecs,rvecs,tvecs,method=cv2.CALIB_HAND_EYE_DANIILIDIS)

homogenousmatrixplaceholder = np.identity(4)
homogenousmatrixplaceholder[0:3,0:3] = rm4
homogenousmatrixplaceholder[0:3,3] = tm4.ravel()
print("Daniilidis Calibration Matrix (4):")
print(homogenousmatrixplaceholder)
print("")

calibrationchoice = int(input("Choose Calibration method (1-4): "))
if(calibrationchoice == 1):
    print("Park-Martin Method")
    cTee = np.identity(4)
    cTee[0:3,0:3] = rm1
    cTee[0:3,3] = tm1.ravel()
    print(cTee)
    
elif(calibrationchoice == 2):
    print("Tsai Method")
    cTee = np.identity(4)
    cTee[0:3,0:3] = rm2
    cTee[0:3,3] = tm2.ravel()
    print(cTee)
    
elif(calibrationchoice == 3):
    print("Horaud Method")
    cTee = np.identity(4)
    cTee[0:3,0:3] = rm3
    cTee[0:3,3] = tm3.ravel()
    print(cTee)
    
elif(calibrationchoice == 4):
    print("Daniilidis Method")
    cTee = np.identity(4)
    cTee[0:3,0:3] = rm4
    cTee[0:3,3] = tm4.ravel()
    
else:
    #default park-martin
    print("Default Park-Martin Method")
    cTee = np.identity(4)
    cTee[0:3,0:3] = rm1
    cTee[0:3,3] = tm1.ravel()
    print(cTee)
    
print("")
    
print("Image Distortion Parameters:")
print(dist)
print("")

print("Camera Intrinsic Matrix:")
print(mtx)
print("")

print("Optimal Camera Intrinsic Matrix")
print(newcameramtx)
print("")
