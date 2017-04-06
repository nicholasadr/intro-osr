import math as m
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-t1","--theta1",type=float,help="link 1 angle value")
ap.add_argument("-t2","--theta2",type=float,help="link 2 angle value")
ap.add_argument("-d1","--link1",type=float,default=0.1,help="link 1 value")
ap.add_argument("-d2","--link2",type=float,default=0.15,help="link 2 value")
args = vars(ap.parse_args())

angle = np.zeros(2)
r = np.zeros(3)
j = np.zeros((len(r),len(angle)))

def fk(a1,a2):
  r[0] = link1*m.cos(a1)+link2*m.cos(a1+a2)
  r[1] = link1*m.sin(a1)+link2*m.sin(a1+a2)
  r[2] = a1+a2
  return r

def jac(a1,a2):
  j[0][0] = -link1*m.sin(a1)-link2*m.sin(a1+a2)
  j[0][1] = -link2*m.sin(a1+a2)
  j[1][0] = link1*m.cos(a1)+link2*m.cos(a1+a2)
  j[1][1] = link2*m.cos(a1+a2)
  j[2][0] = 1
  j[2][1] = 1
  return j


if __name__ == '__main__':
  angle[0] = args["theta1"]
  angle[1] = args["theta2"]
  link1 = args["link1"]
  link2 = args["link2"]
  fk(angle[0],angle[1])
  jac(angle[0],angle[1])
  print "x: ",r[0]
  print "y: ",r[1]
  print "z: ",r[2]
  print "jacobian: ",j
