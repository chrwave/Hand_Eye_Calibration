import socket
import time
import numpy as np
import robot
import robotconfig as rcfg
from robot_class import Robot
from gripper import Gripper
if  rcfg.grippername=='robotiq': 
  import robotiq as gripper
if  rcfg.grippername=='rg2': 
  import rg2 as gripper
robotfunc = Robot()
gripperfunc = Gripper()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
##########################################################################################
#
# Insert your function definitions here 

    
HOST1 = rcfg.hostip    
PORT1 = 30003              
s.connect((HOST1, PORT1))
#gripper.init(s)
time.sleep(1)
########################################################################################## 
# Enter your calibration coordinates here
# 
gripperfunc.open()
gripperfunc.wait()

robot.transform_init([-390.3,350.6, 31.0],[-394, 166.4,31.0],[-245,347,27.6])
x=0.1
y=0.15
z=0.02
rx=rcfg.homeangle[0]
ry=rcfg.homeangle[1]
rz=rcfg.homeangle[2]

t=robot.transform(x,y,z)
s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')
robotfunc.wait()
gripperfunc.close()
gripperfunc.wait()

x=0.05
y=0.05
z=0.02
t=robot.transform(x,y,z)
s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')

robotfunc.wait()
s.close()
robotfunc.shutdown()
