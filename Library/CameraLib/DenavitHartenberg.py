# -*- coding: utf-8 -*-
"""
Created on Thu Apr 22 14:40:35 2021

@author: Christian & Andreas
"""

import numpy as np

def DenavitHartenbergUR(theta,UR):
    # The kinematics for the UR based on parameters from
    # https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    # Theta is a list of all six joint angles
    # UR is the arm: UR3, UR5, UR10
    
    # Initialize the parameters for UR3
    if(UR == "UR3"):
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
        alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        T06 = DenavitHartenbergMatrix(a[0], d[0], alpha[0], theta[0]) # Initialize with the first
    
    # Initialize the parameters for UR5
    elif(UR == "UR5"):
        a = [0, -0.425, -0.39225, 0, 0, 0]
        d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
        alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        T06 = DenavitHartenbergMatrix(a[0], d[0], alpha[0], theta[0]) # Initialize with the first
    
    # Initialize the parameters for UR10
    elif(UR == "UR10"):
        a = [0, -0.612, -0.5723, 0, 0, 0]
        d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]
        alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        T06 = DenavitHartenbergMatrix(a[0], d[0], alpha[0], theta[0]) # Initialize with the first
    
    else:
        print("Error, unknown UR")
        return "Error!"
    
    for i in range(5):
        j = i+1
        # print(j)
        T06 = np.matmul(T06,DenavitHartenbergMatrix(a[j], d[j], alpha[j], theta[j]))
    
    return T06
    

# The Denavit Hertenberg Matrix
def DenavitHartenbergMatrix(a,d,alpha,th):
    T = [[np.cos(th), -np.sin(th)*np.cos(alpha),  np.sin(th)*np.sin(alpha), a*np.cos(th)],
         [np.sin(th),  np.cos(th)*np.cos(alpha), -np.cos(th)*np.sin(alpha), a*np.sin(th)],
         [         0,             np.sin(alpha),             np.cos(alpha),            d],
         [         0,                         0,                         0,            1]]
    return np.array(T)


# Test with a random list of joint angles

# the = [0, np.pi/4, 0, 0, np.pi/2, 0]
# T06 = DenavitHartenbergUR(the,"UR5")
# print(T06)

