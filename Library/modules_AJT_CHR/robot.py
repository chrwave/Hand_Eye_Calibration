import numpy as np
Arot=np.array([[0 ,0,0],[0,0,0],[0,0,0]])
otask=0
def transform_init(p0i,pxi,pyi):
  global Arot
  global otask
  p0=np.array(p0i)
  px=np.array(pxi)
  py=np.array(pyi)
  p0=p0/1000.
  px=px/1000.
  py=py/1000.
  vx=px-p0
  vy=py-p0
  vx=vx/np.linalg.norm(vx)
  vy=vy/np.linalg.norm(vy)
  vz=np.cross(vx,vy)
  Arot=np.array([ vx,vy,vz])
  Arot=np.transpose(Arot)
  otask=p0
  
def transform(x,y,z):
  b=np.array([x,y,z])
  t=Arot.dot(np.transpose(b))+otask
  return t
  



