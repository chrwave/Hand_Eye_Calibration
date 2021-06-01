import socket

gs = 0;

def init(s):
  global gs
  gs = gs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  gs.connect(('localhost', 31383))
  activate()
  
def activate():
  global gs
  gs.send('rq_activate()')

def open():
  global gs
  gs.send('rq_open()')

def close():
  global gs
  gs.send('rq_close()')

def move(val):
  global gs
  gs.send('rq_move(' + str(val) + ')')
