import openravepy as orpy
import numpy as np
from scipy.linalg import expm

env = orpy.Environment()
env.SetViewer('qtcoin')
robot = env.ReadRobotXMLFile('../../../robots/denso_robotiq_85_gripper.robot.xml')
env.AddRobot(robot)
manipulator = robot.SetActiveManipulator('denso_robotiq_85_gripper')

q1 = np.array([-0.1, 1.8, 1.0, 0.5, 0.2, 1.3, 0])
qdot1 = np.array([1.2, -0.7, -1.5, -0.5, 0.8, -1.5, 0])
delta_t = 0.1

robot.SetDOFValues(q1)
tool_tip_origin = robot.GetLinks()[19].GetTransform()[:3,3]

T1 = manipulator.GetTransform()
print "T1"
print T1

Jlin = robot.ComputeJacobianTranslation(19,tool_tip_origin)
print "Linear Jacobian"
print Jlin

Jang = robot.ComputeJacobianAxisAngle(19)
print "Angular Jacobian"
print Jang

robot.SetDOFValues(q1+delta_t*qdot1)
T2 = manipulator.GetTransform()
print "T2"
print T2

X1 = T1[:3,3]
R1 = T1[:3,:3]

v = np.dot(Jlin,qdot1)
X2 = X1+(delta_t*v)

w = np.dot(Jang,qdot1)
w_x = np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
R2 = np.dot(expm(delta_t*w_x),R1)

T2_j = np.zeros((4,4))
T2_j[:3,3] = X2
T2_j[:3,:3] = R2
T2_j[3,3] = 1

print "Difference between T2_j and T2: "
print T2_j-T2
