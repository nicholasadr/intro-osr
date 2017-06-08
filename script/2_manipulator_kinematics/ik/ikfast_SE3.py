'''
IKFAST INVERSE KINEMATICS IMPLEMENTATION WITH INTERPOLATION IN SE(3)
by @nicholasadr

Interpolation is based on chapter II in "Time-optimal path parameterization of rigid-body motions: applications to spacecraft reorientation" by Huy Nguyen and Quang-Cuong Pham
http://www.ntu.edu.sg/home/cuong/docs/TOPPSO3SE3.pdf

RUN:
python task_vel_space_multi_ik.py -s {steps to divide the movement into} 
'''

import numpy as np
import openravepy as orpy
import argparse
import math
import copy
from scipy.linalg import logm,expm

ap = argparse.ArgumentParser()
ap.add_argument("-s","--step",type=int,help="no of steps to break the path into")
args = vars(ap.parse_args())

env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('../../../envs/grasp.xml')
robot = env.GetRobots()[0]
manipulator = robot.SetActiveManipulator('denso_robotiq_85_gripper')

#get current end-effector pose
T_start = manipulator.GetTransform()
R_start = manipulator.GetTransform()[:3,:3]
x_start = manipulator.GetTransform()[:3,3]

#end-effector goal pose
T_end = np.array([[0,1,0,0.5], [0,0,1,0.3], [1,0,0,0.4], [0,0,0,1]])
R_end = np.array([[0,1,0],[0,0,1],[1,0,0]])
x_end = np.array([[0.5,0.3,0.4]])

t = 1.0/args["step"]
ti = 0

#Interpolate in SO(3)
#assuming w(0)=0 and w(1)=0

#finding r1
r1 = np.matmul(np.transpose(R_start),R_end)  #r1 = log(R_start transpose * R_end)
eps = 1e-50
r1[r1==0]=eps   #replace zero with very small value
r1 = logm(r1)

a2 = copy.deepcopy(r1)
a2[0][2]*=3
a2[2][0]*=3
a3 = copy.deepcopy(r1)
a3[0][2]*=-2
a3[2][0]*=-2

#finding k3,k2,k1,k0 assuming v(0)=v(1)=0
k0=x_start
k2=np.multiply(3,(x_end-x_start))
k3=np.multiply(-2,(x_end-x_start))

T = np.zeros((4,4))
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
  ikmodel.autogenerate()

#loop
for i in range(args["step"]+1):
  print "  "
  print "  "
  print "===================== STEP ",i+1
  #get R
  rw=np.dot(a3,math.pow(ti,3))+np.dot(a2,math.pow(ti,2))
  rw2=np.dot(a3,math.pow(ti+t,3))+np.dot(a2,math.pow(ti+t,2))
  Rt1=np.dot(R_start,expm(rw))
  print "R : ",Rt1

  #get x
  pw=np.dot(k3,math.pow(ti,3))+np.dot(k2,math.pow(ti,2))+k0
  print "x: ",pw

  ti+=t

  T[:3,3] = pw 
  T[:3,:3] = Rt1
  T[3,3] = 1
  #print "T: ",T
  solutions = manipulator.FindIKSolutions(T,0)
  #print solutions
  robot.SetDOFValues(np.hstack([solutions[1],[0]]))
  raw_input("Press ENTER for next task-space with velocity-space IK step")
