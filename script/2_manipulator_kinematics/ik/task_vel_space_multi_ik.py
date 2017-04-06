'''
[IN PROGRESS]
TASK-SPACE IK WITH VELOCITY-SPACE ON MULTIPLE AXIS"
by @nicholasadr
Code is meant for http://osrobotics.org/pages/inverse_kinematics.html exercise
Task-space IK using velocity-space IK for comparison with [Velocity-space IK with OpenRAVE]

RUN:
python task_vel_space_multi_ik.py -s {steps to divide the movement into} 
'''

import numpy as np
import openravepy as orpy
import argparse
import copy
from scipy.linalg import logm

ap = argparse.ArgumentParser()
ap.add_argument("-s","--step",type=int,help="no of steps to break the path into")
#ap.add_argument("-z","--z",type=float,help="height in z axis to lift the panel")
args = vars(ap.parse_args())

env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('../envs/grasp.xml')
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
p_start = manipulator.GetTransform()

#end-effector goal pose
p_end = np.array([[0,1,0,0.5], [0,0,1,0.3], [1,0,0,0.4], [0,0,0,1]])
#p_end = copy.deepcopy(p_start)
#p_end[2]+=[0,0,0,args["z"]]

#break down path into smaller paths and creating combined list of all intermediate poses
p_delta = ((p_end-p_start)/float(args["step"]))[:3,3]
p_delta = np.append(p_delta,[0, 0, 0])
print "p_delta",p_delta
r_delta = p_end[:3,:3]-p_start[:3,:3]
print "r_delta",r_delta
print "P1",p_start[:3,:3]
print "P2",p_end[:3,:3]
omega_mat = np.zeros((3,3))
with np.errstate(divide='ignore',invalid='ignore'):
  a = np.divide(p_end[:3,:3],p_start[:3,:3])
  print "a",a
  #a = np.nan_to_num(a)
  a[a == -np.inf] = 0
  a[np.isnan(a)] = 0
  print "a2",a

  #omega_mat = logm(np.nan_to_num(np.divide(p_end[:3,:3],p_start[:3,:3])))
  omega_mat = logm(a)
print "omega_mat",omega_mat
omega = np.zeros((1,3))
omega[0,0]=omega_mat[2,1]
omega[0,1]=omega_mat[0,2]
omega[0,2]=omega_mat[1,0]
print "omega",omega
#omega[omega == -np.inf] = 0
pdot_des = p_delta + np.insert(np.dot(omega,r_delta),0,[0,0,0])
print "pdot_des",pdot_des
#pdot_des = np.array([0, 0, p_delta, 0, 0, 0])

#for loop
for i in range(args["step"]):
  #get J matrix for current pose
  J[:3,:] = robot.ComputeJacobianTranslation(8, robotiq_base_origin)[:,:6]
  J[3:,:] = robot.ComputeJacobianAxisAngle(8)[:,:6]
  qdot = np.linalg.solve(J,pdot_des)
  q[:6]+=qdot
  #robot.SetDOFValues(q[i+1])
  robot.SetDOFValues(q)
  raw_input("Press ENTER for next task-space with velocity-space IK step")
