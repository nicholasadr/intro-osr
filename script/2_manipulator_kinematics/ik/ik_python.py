import math as m
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-x","--x1",type=float,help="x coordinate")
ap.add_argument("-y","--y1",type=float,help="y coordinate")
ap.add_argument("-d1","--link1",type=float,default=0.1,help="link 1 value")
ap.add_argument("-d2","--link2",type=float,default=0.15,help="link 2 value")
args = vars(ap.parse_args())

goal = np.zeros(2)
r = np.zeros(4)

def ik(a1,a2):
  r[0] = m.acos(((goal[0]*goal[0])+(goal[1]*goal[1])-(link1*link1)-(link2*link2))/(2*link1*link2))
  r[1] = r[0]*-1
  r[2] = m.atan2(goal[1],goal[0])-m.atan2(link2*m.sin(r[0]),link1+link2*m.cos(r[0]))
  r[3] = m.atan2(goal[1],goal[0])-m.atan2(link2*m.sin(r[1]),link1+link2*m.cos(r[1]))
  return r

if __name__ == '__main__':
  goal[0] = args["x1"]
  goal[1] = args["y1"]
  link1 = args["link1"]
  link2 = args["link2"]
  ik(goal[0],goal[1])
  print "Answer 1 :"
  print "theta1: ",r[2]
  print "theta2: ",r[0]
  print "Answer 2 :"
  print "theta1: ",r[3]
  print "theta2: ",r[1]
