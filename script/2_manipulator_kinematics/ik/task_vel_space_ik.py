'''
[IN PROGRESS]
TASK-SPACE IK WITH VELOCITY-SPACE IK"
by @nicholasadr
Code is meant for http://osrobotics.org/pages/inverse_kinematics.html exercise
Task-space IK using velocity-space IK with non-constant orientation 

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
#grasping_config = [0.1623582, 0.89821769, 1.17244704, 1.64914659, 1.42844832, 2.63613546, 0]
#robot.SetDOFValues(grasping_config)
#robot.Grab(env.GetKinBody('thinbox'))
robotiq_base_origin = robot.GetLinks()[8].GetTransform()[:3,3]

J = np.zeros((6,6))

#get current q pose
q = robot.GetDOFValues()

#get current end-effector goal pose
T_start = manipulator.GetTransform()
R_start = manipulator.GetTransform()[:3,:3]
x_start = manipulator.GetTransform()[:3,3]

#end-effector goal pose
T_end = np.array([[0,1,0,0.5], [0,0,1,0.3], [1,0,0,0.4], [0,0,0,1]])
R_end = np.array([[0,1,0],[0,0,1],[1,0,0]])
x_end = np.array([[0.5,0.3,0.4]])

t = 1.0/args["step"]
ti = 0
#============================================================================================
#Interpolate linear velocity to intermediate linear velocities
#get v or linear velocity
#x_delta = (x_end-x_start)/args["step"]

#=============================================================================================
#Interpolate transformation matrix between R_start and R_end in SO(3) to R(t), t[0,1]
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
#=============================================================================================
#loop
for i in range(args["step"]):
  #get J matrix for current pose
  J[:3,:] = robot.ComputeJacobianTranslation(8, robotiq_base_origin)[:,:6]
  J[3:,:] = robot.ComputeJacobianAxisAngle(8)[:,:6]
  J[3,3]=eps
  #J[J==0] = eps

  #get R_delta
  rw=np.dot(a3,math.pow(ti,3))+np.dot(a2,math.pow(ti,2))
  #print "rw : ",rw
  rw2=np.dot(a3,math.pow(ti+t,3))+np.dot(a2,math.pow(ti+t,2))
  #print "rw2 : ",rw2
  Rt1=np.dot(R_start,expm(rw))
  print "Rt1 : ",Rt1
  Rt2=np.dot(R_start,expm(rw2))
  print "Rt2 : ",Rt2

  #get w
  Rt1[Rt1==0]=eps
  Rt2[Rt2==0]=eps
  w_x=logm(Rt2)-logm(Rt1)
  print "w_x : ",w_x

  #get p_delta
  pw=np.dot(k3,math.pow(ti,3))+np.dot(k2,math.pow(ti,2))+k0
  print "pw: ",pw
  pw2=np.dot(k3,math.pow(ti+t,3))+np.dot(k2,math.pow(ti+t,2))+k0
  print "pw2: ",pw2

  #get v
  v = pw2-pw
  print "v: ",v

  ti+=t

  #get w from w_x and
  #combine v and w to pdot_des
  pdot_des = np.append(v,[w_x[2][1],w_x[0][2],w_x[1][0]])
  #print "x_delta : ",x_delta
  print "pdot_des : ",pdot_des
  print "J : ",J
  qdot = np.linalg.solve(J,pdot_des)
  print "qdot : ",qdot
  q[:6]+=qdot
  robot.SetDOFValues(q)
  raw_input("Press ENTER for next task-space with velocity-space IK step")
