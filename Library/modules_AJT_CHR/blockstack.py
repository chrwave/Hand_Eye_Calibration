import socket
import time
import numpy as np
import robot
import sys
#import rg2 as gripper
from robot_class import Robot
from gripper import Gripper
import config_robot as cfg
try:
    robot_control = Robot()
    gripper_control = Gripper()
except KeyboardInterrupt:
    robot_control.shutdown()
    sys.exit("KeyboardInterrupt")
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  
def goto(x,y,z):
  t=robot.transform(x,y,z)
  robot_control.send_line('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+',2.22,-2.22,0],1,0.1)\n')
  
def pick_at(x,y,z):
  gripper_control.open()
  goto(x,y,z+0.05)
  robot_control.wait()
  goto(x,y,z)
  robot_control.wait()
  gripper_control.close()
  gripper_control.wait()
  goto(x,y,z+0.05)
  robot_control.wait()
  
def place_at(x,y,z):
  goto(x,y,z+0.05)
  robot_control.wait()
  goto(x,y,z)
  robot_control.wait()
  gripper_control.open()
  gripper_control.wait()
  goto(x,y,z+0.05)
  robot_control.wait()
  
#s.connect((cfg.SOCKETS["host ip"], cfg.SOCKETS["port send"]))
#time.sleep(2)
try:
    robot.transform_init([-390.3, 350.6, 31.0], [-394.0, 166.4, 31.0], [-245.0, 347.0, 27.6])
    x=0.1
    y=0.15
    z=0.15
    t=robot.transform(x,y,z)
    x=t[0]
    y=t[1]
    z=t[2]
    robot_control.send_line('movel(p['+str(x)+','+str(y)+','+str(z)+',2.22,-2.22,0],1,0.1)\n')
    robot_control.wait()
    targetx=0.20
    targety=0.15
    targetz=0
    blockh=0.03
    pick_at(0.05,0,-0.0)
    place_at(targetx,targety,targetz)
    pick_at(0.10,0,-0.0)
    place_at(targetx,targety,blockh+targetz)
    pick_at(0.15,0,-0.0)
    place_at(targetx,targety,2*blockh+targetz)
    pick_at(targetx, targety, 2*blockh+targetz)
    place_at(0.15,0,0)
    pick_at(targetx, targety, blockh+targetz)
    place_at(0.1,0,0)
    pick_at(targetx, targety, targetz)
    place_at(0.05, 0 ,0)
    robot_control.wait()
    robot_control.shutdown()
    gripper_control.shutdown()
except KeyboardInterrupt:
    robot_control.shutdown()

#s.close()

