# -*- coding: utf-8 -*-
"""
Created on Thu Apr 22 14:40:35 2021

@author: Christian & Andreas
"""

import numpy as np
import cv2

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


def InvKinematic(x, y, z, rx, ry, rz):
    th = np.zeros((6,8))
    
    d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])#ur5   
    #print(d)
    a = np.array([0 ,-0.425 ,-0.39225 ,0 ,0 ,0])#ur5 mm
    alph = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0 ]) # ur5

    rvec = np.array([rx,ry,rz])
    tvec = np.array([x,y,z])
    rotation,__ = cv2.Rodrigues(rvec)
    desired_pos = np.identity(4)
    desired_pos[0:3,0:3] = rotation
    desired_pos[0:3,3] = tvec
    
    # BASE JOINT
    # First we must find the location of frame 5 in relation to the base frame
    P50 = desired_pos @ np.array([0,0,-d[5],1])
    #print(P50)

    # Length length from base to P50 xy-plane
    LP50 = np.sqrt(P50[0]**2+P50[1]**2)
    # Two solutions, Shoulder can either be "left" or "right"
    try:
        th[0,0:4] = np.arctan2(P50[1],P50[0]) + np.arccos(d[3]/LP50)+np.pi/2
    except:
        th[0,0:4] = np.nan
    try:
        th[0,4:8] = np.arctan2(P50[1],P50[0]) - np.arccos(d[3]/LP50)+np.pi/2
    except:
        th[0,4:8] = np.nan
        
    # WRIST 2
    # Two solutions exists for the wrist, it can either be up or down. Recall that it depends on the angle of base joint, therefore we must use this.
    cl = [0,4]
    for i in range (0,len(cl)):
        c = cl[i]
        t =  np.arccos(( desired_pos[0,3]*np.sin(th[0,c]) - desired_pos[1,3]*np.cos(th[0,c]) - d[3]) / d[5])
        th[4,c:c+2] = t
        th[4,c+2:c+4] = -t 
        
    # WRIST 3
    cl = [0,2,4,6]
    invdespos = np.linalg.inv(desired_pos)
    for i in range (0,len(cl)):
        c = cl[i]
        th[5,c:c+2] = np.arctan2((-invdespos[1,0]*np.sin(th[0,c])+invdespos[1,1]*np.cos(th[0,c]))/np.sin(th[4,c]), (invdespos[0,0]*np.sin(th[0,c])-invdespos[0,1]*np.cos(th[0,c]) )/np.sin(th[4,c]))
    
    # ELBOW
    # Two solutions exist, elbow up and elbow down
    # At this point T10, T45 and T56 are known.
    cl = [0,2,4,6]
    for i in range (0,len(cl)):
        c = cl[i]
        T01 = DenavitHartenbergMatrix(a[0],d[0],alph[0],th[0,c]) #self.dh_par(th[0,c],self.a1,self.d1,self.alpha1)
        T45 = DenavitHartenbergMatrix(a[4],d[4],alph[4],th[4,c])#self.dh_par(th[4,c],self.a5,self.d5,self.alpha5)
        T56 = DenavitHartenbergMatrix(a[5],d[5],alph[5],th[5,c])#self.dh_par(th[5,c],self.a6,self.d6,self.alpha6)
        T10 = np.linalg.inv(T01)
        T54 = np.linalg.inv(T45)
        T65 = np.linalg.inv(T56)

        T14 = (T10 @ desired_pos) @ (T65 @ T54)
        LP41 = np.sqrt(T14[0,3]**2+T14[1,3]**2) # Maybe use y instead of z
        try:
            t = np.arccos((LP41**2 - a[1]**2 - a[2]**2 )/(2*a[1]*a[2]))
        except:
            t = np.nan
        th[2,c] = t
        th[2,c+1] = -t
        
    # SHOULDER AND WRIST 1
    for i in range (0,8):
        c = i
        #print(c)
        T01 = DenavitHartenbergMatrix(a[0],d[0],alph[0],th[0,c])
        T45 = DenavitHartenbergMatrix(a[4],d[4],alph[4],th[4,c])
        T56 = DenavitHartenbergMatrix(a[5],d[5],alph[5],th[5,c])
        T10 = np.linalg.inv(T01)
        T54 = np.linalg.inv(T45)
        T65 = np.linalg.inv(T56)

        T14 = (T10 @ desired_pos) @ (T65 @ T54)
        LP41 = np.sqrt(T14[0,3]**2+T14[1,3]**2)

        th[1,c] = np.arctan2(-T14[1,3],-T14[0,3]) - np.arcsin((-a[2]*np.sin(th[2,c]))/LP41)
        #print(th[1,c])
        T21 = DenavitHartenbergMatrix(a[1],d[1],alph[1],th[1,c])#self.dh_par(th[1,c],self.a2,self.d2,self.alpha2)
        T32 = DenavitHartenbergMatrix(a[2],d[2],alph[2],th[2,c])#self.dh_par(th[2,c],self.a3,self.d3,self.alpha3)
        T12 = np.linalg.inv(T21)
        T23 = np.linalg.inv(T32)

        T34 = T23 @ T12 @ T14
        th[3,c] = np.arctan2(T34[1,0], T34[0,0])
        #print(th[1,c])
    
    return (th,(th * 180/np.pi))


def InvKinematicOptimal(x,y,z,rx,ry,rz,curr_jointposition):
    # Inverse kinematics:
    # InvKinematic(x, y, z, rx, ry, rz)
    invK = InvKinematic(x,y,z,rx,ry,rz)
    
    
    # The inverse kinematics is now found and the optimal joint position must be found
    ############################################
    ############################################
    
    # INVERSE KINEMATICS OPTIMAL SOLUTION FINDER!
    
    ############################################
    ############################################
    
    invK_copy = invK[0].copy()
    invK_diff = invK_copy.copy()
    
    # Loop every row of the inverse kinematics
    for idx1 in range(6):
        row = invK_copy[idx1]
        row_low_temp = np.zeros([3,1])
        row_low_temp2 = np.zeros([3,1])
        curr_val = curr_jointposition[idx1]    
        
        # Loop every entry of the row
        for idx2 in range(8):
            # Calculate the error from the current value
            row_low_temp[0] = curr_val - row[idx2]
            row_low_temp[1] = curr_val - (row[idx2] + (np.pi*2))
            row_low_temp[2] = curr_val - (row[idx2] - (np.pi*2))
            
            
            row_low_temp2[0] = row[idx2]
            row_low_temp2[1] = (row[idx2] + (np.pi*2))
            row_low_temp2[2] = (row[idx2] - (np.pi*2))
            
            # print("rowLowtemp")
            # print(row_low_temp2)
            
            # Take the absolute value of the error
            abs_row_low = abs(row_low_temp)
            print("abs_row_low")
            print(abs_row_low)
            
            # Find the index of the lowest absolute value
            low_idx = np.where(abs_row_low == np.amin(abs_row_low))
            
            # Insert the non-absolute value in the row instead:
            try:
                invK_copy[idx1][idx2] = row_low_temp2[low_idx[0][0]][low_idx[1][0]]
                invK_diff[idx1][idx2] = curr_val - row_low_temp2[low_idx[0][0]][low_idx[1][0]]
            except:
                invK_copy[idx1][idx2] = 999999
                invK_diff[idx1][idx2] = 999999
    
    # The matrix invK_copy does now contain the smallest rotations
    
    # Now, each of the eight solutions are evaluated based on their ranking:
    
    score_mat = np.zeros([len(invK_copy),8])
        
    # Loop every row
    for idx1 in range(6):
        row = invK_copy[idx1]
        # Find the smallest value on the row and assign 1 to the score.
        # If more than one has the same (smallest) value, all will be
        # assigned with 1.
        curr_val = curr_jointposition[idx1]

        abs_row = abs(row)
        
        # This line finds all of the smallest values and returns multiple indices
        # if more than one has the same (smallest) value:
        if(idx1 == 0):

            low_value_idx = np.where(abs_row == np.amin(abs_row))

        else:
            low_value_idx = np.where(abs_row[0:4] == np.amin(abs_row[0:4]))
            low_value_idx2 = np.where(abs_row[4:9] == np.amin(abs_row[4:9]))
    
        # For every lowest value assign a score of 1
        if(idx1 == 0):
            for idx2 in low_value_idx:
                score_mat[idx1][idx2] = 1
        else:
            for idx2 in low_value_idx:
                score_mat[idx1][idx2] = 1
            
            for idx2 in low_value_idx2:
                score_mat[idx1][idx2+4] = 1
        
    # From the score system, find the solution with the most points:
    total_score = np.zeros(8)
    
    for idx1 in range(8):
        column = score_mat[:,idx1]
        total_score[idx1] = np.sum(column)
    
    # Find the solution index with the highest score:
    invK_solution_nr = np.where(total_score == np.amax(total_score))
    invK_solution_nr = invK_solution_nr[0][0]
    
    # Corresponding to the inverse solution:
    invK_solution = invK_copy[:,invK_solution_nr]
    print("Optimal inverse kinematics solution:")
    print(invK_solution)
    print(np.rad2deg(invK_solution))
    
    return invK_solution, invK_copy, score_mat