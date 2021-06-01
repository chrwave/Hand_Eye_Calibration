import socket
import time
import numpy as np
import robot
import robotconfig as rcfg
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

HOST1=rcfg.hostip
PORT1 = 30003              # The same port as used by the server
s.connect((HOST1, PORT1))
x=rcfg.homepos[0]
y=rcfg.homepos[1]
z=rcfg.homepos[2]
rx=rcfg.homeangle[0]
ry=rcfg.homeangle[1]
rz=rcfg.homeangle[2]
s.send('movel(p['+str(x)+','+str(y)+','+str(z)+','+str(rx)+','+str(ry)+','+str(rz)+']' +',1,0.1)\n')
time.sleep(6)
s.close()

