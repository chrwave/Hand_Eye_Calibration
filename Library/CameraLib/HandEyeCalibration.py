# -*- coding: utf-8 -*-
"""
Created on Mon Apr 19 09:17:02 2021

@author: Andre
"""

import numpy as np
from math import *
#import mpmath
#import scipy

def logMatrix(R):
    # Rotation matrix logarithm (Not the same as logm)
    theta = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1.0)/2.0)
    return np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]) * theta / (2*np.sin(theta))

def skew(vector):
    return np.float64(np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]]))

def TsaiHandEyeCalibration(A,B):
    #Tsai Hand-Eye Calibration
    n = len(A)
    S = np.zeros((3*n,3))
    V = np.zeros((3*n))
    
    for i in range(n):
        A1 = mpmath.logm(A[i][0:3,0:3])
        #print(A1)
        B1 = mpmath.logm(B[i][0:3,0:3])
        a = np.float64(np.array([A1[2,1], A1[0,2], A1[1,0]])).T
        a = a/np.linalg.norm(a)
        b = np.float64(np.array([B1[2,1], B1[0,2], B1[1,0]])).T
        b = b/np.linalg.norm(b)
       # print(i)
        S[3*i:3*(i+1),:] = skew(a+b)
        #print(a-b)
        V[3*i:3*(i+1)] = a-b;
        
    
    V = V.reshape(6,1)
    x,resid,rank,s = np.linalg.lstsq(S,V)
    theta = 2*np.arctan(np.linalg.norm(x));
    x = x/np.linalg.norm(x);
    R = (np.eye(3)*np.cos(theta) + np.sin(theta)*skew(x) + ((1-np.cos(theta))*x)*x.T).T;
    
    C = np.zeros((3*n,3))
    d = np.zeros((3*n));
    I = np.eye(3);
    for i in range(n):
        C[3*i:3*(i+1),:] = I - A[i][0:3,0:3]
        d[3*i:3*(i+1)] = A[i][0:3,3]-np.dot(R,B[i][0:3,3])
    
    t,resid,rank,s = np.linalg.lstsq(C,d)
    t = t.reshape(3,1)
    Bottomrowmatrix = np.array([0, 0, 0, 1])
    Bottomrowmatrix = Bottomrowmatrix.reshape(1,4)
    X = np.concatenate((R,t),axis=1)
    X = np.concatenate((X,Bottomrowmatrix),axis=0)
    
    return X
    


#def skew(vector):
#    X = np.array([[0, -vector[2], vector[1]],
#                  [vector[2], 0, -vector[0]],
#                  [-vector[1], vector[0], 0]])
#    return X
#        