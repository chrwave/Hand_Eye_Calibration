import socket
import time
import sys
sys.path.append('../config')
sys.path.append('/shome/31383/k383teach/modules1')
import robotconfig
from gripper import Gripper
gripperfunc = Gripper() 
gripperfunc.close()
gripperfunc.wait()



