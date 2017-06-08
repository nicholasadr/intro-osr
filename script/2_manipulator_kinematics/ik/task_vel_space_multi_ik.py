'''
TASK-SPACE IK WITH VELOCITY-SPACE - TRANSLATION ON MULTIPLE AXIS (CONSTANT ORIENTATION)"
by @nicholasadr
Code is meant for http://osrobotics.org/pages/inverse_kinematics.html exercise
Task-space IK using velocity-space IK for comparison with [Velocity-space IK with OpenRAVE] with translation in multiple axis (constant orientation)

RUN:
python task_vel_space_single_ik.py -s {steps to divide the movement} -x {movement distance in x-axis} -y {movement distance in y-axis} -z {movement distance in z-axis}
'''

import numpy as np
import openravepy as orpy
import argparse
import copy

ap = argparse.ArgumentParser()
ap.add_argument("-s","--step",type=int,help="no of steps to break the path into")
ap.add_argument("-x","--x",type=float,help="distance in x axis")
ap.add_argument("-y","--y",type=float,help="distance in y axis")
ap.add_argument("-z","--z",type=float,help="distance in z axis")
args = vars(ap.parse_args())

env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('../../../envs/grasp.xml')
robot = env.GetRobots()[0]
manipulator = robot.SetActiveManipulator('denso_robotiq_85_gripper')
grasping_config = [0.1623582, 0.89821769, 1.17244704, 1.64914659, 1.42844832, 2.63613546, 0]
robot.SetDOFValues(grasping_config)
robot.Grab(env.GetKinBody('thinbox'))
robotiq_base_origin = robot.GetLinks()[8].GetTransform()[:3,3]

J = np.zeros((6,6))

#get current q pose
q = robot.GetDOFValues()

#get current end-effector pose
p_start = manipulator.GetTransform()

#end-effector goal pose
p_end = copy.deepcopy(p_start)
p_end[:3,3]+=[args["x"],args["y"],args["z"]]

#break down path into smaller paths and find the next linear velocity
x_delta = ((p_end-p_start)/float(args["step"]))[0,3]
y_delta = ((p_end-p_start)/float(args["step"]))[1,3]
z_delta = ((p_end-p_start)/float(args["step"]))[2,3]
pdot_des = np.array([x_delta, y_delta, z_delta, 0, 0, 0])

#move
for i in range(args["step"]):
  #get J matrix for current pose
  J[:3,:] = robot.ComputeJacobianTranslation(8, robotiq_base_origin)[:,:6]
  J[3:,:] = robot.ComputeJacobianAxisAngle(8)[:,:6]
  qdot = np.linalg.solve(J,pdot_des)
  q[:6]+=qdot
  #robot.SetDOFValues(q[i+1])
  robot.SetDOFValues(q)
  raw_input("Press ENTER for next task-space with velocity-space IK step")
