# -*- coding: utf-8 -*-
"""
Created on Tue May  4 09:46:16 2021

@author: chris & Andre
"""
import numpy as np
import cv2


def redimage(img):

    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow('red',img_hsv)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    # lower_red = np.array([0,100,100])
    # upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    
    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    # lower_red = np.array([160,100,100])
    # upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    
    # join my masks
    mask = mask0+mask1
    
    # set my output img to zero everywhere except my mask
    output_img = img.copy()
    output_img[np.where(mask==0)] = 0
    
    # or your HSV image, which I believe is what you want
    output_hsv = img_hsv.copy()
    output_hsv[np.where(mask==0)] = 0
    
    cv2.imshow('redimage',output_hsv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    #output_hsv = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    
    return output_hsv


    
src = cv2.imread(r"D:\Scripts\PythonScripts\dots2.PNG")
# cv2.imshow('image',src )
# cv2.waitKey(0)
# cv2.destroyAllWindows()


#gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)


#gray = cv2.medianBlur(gray, 5)

red_img = redimage(src)

#red_img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
# cv2.imshow('gray',red_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)
# cv2.imshow('gray',gray)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# gray = cv2.medianBlur(gray, 5)
# gray = cv2.GaussianBlur(gray, (5,5), 0)

red_img = cv2.medianBlur(red_img, 7)
cv2.imshow('red_img',red_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
red_img = cv2.GaussianBlur(red_img, (7,7), 0)
cv2.imshow('red_img',red_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

rows = gray.shape[0]


circles = cv2.HoughCircles(red_img[:,:,2], cv2.HOUGH_GRADIENT, 1, rows / 8,
                           param1=100, param2=30,
                           minRadius=1, maxRadius=70)


if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        center = (i[0], i[1])
        # circle center
        cv2.circle(src, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        cv2.circle(src, center, radius, (255, 0, 255), 3)
        print(center)


cv2.imshow("detected circles", src)
cv2.waitKey(0)
cv2.destroyAllWindows()