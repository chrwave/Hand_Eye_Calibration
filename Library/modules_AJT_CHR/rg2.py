import socket
import time
gs=0
def init(s):
  global gs
  gs=s
  

def open():
  global gs
  gs.send('set_digital_out(8,False) \n')
  time.sleep(0.2)

def close():
  global gs
  gs.send('set_digital_out(8,True) \n')
  time.sleep(0.2)

