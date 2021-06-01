import socket
import time
import sys
sys.path.append('../config')
sys.path.append('D:\Scripts\PythonScripts\modules_AJT_CHR')
import robotconfig
from src.ur.class_ur import UR
from src.gripper.class_gripper import Gripper
import robot
import robotconfig as rcfg

HOST1 = rcfg.HOST_IP
PORT1 = 30003              # The same port as used by the server

ur = UR(HOST1, PORT1)
gripperfunc = Gripper() 
gripperfunc.open()
gripperfunc.wait()



