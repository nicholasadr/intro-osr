#! /usr/bin/env python
import rospy
import copy
import actionlib
import os
import time
import collections
import PyKDL
import criros
import numpy as np
import openravepy as orpy
import matplotlib.pyplot as plt
import tf.transformations as tr
from tf_conversions import posemath
from control_msgs.msg import *
from trajectory_msgs.msg import *
from robotiq_control import controller
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

#ReadJointPosition Class: reading the position of joints from Gazebo 
class ReadJointPosition(object):
  def __init__(self, namespace, timeout):
    self.ns = criros.utils.solve_namespace(namespace)
    # Set-up subscriber -> infomation about current positions of robot's joints will be provided
    self._js_sub = rospy.Subscriber('%sjoint_states' % self.ns, JointState, self.joint_states_cb, queue_size=1)
    rospy.logdebug('Waiting for [%sjoint_states] topic' % self.ns)
    start_time = rospy.get_time()
    while not hasattr(self, '_joint_names'):
      if (rospy.get_time() - start_time) > timeout:
        rospy.logerr('Timed out waiting for joint_states topic')
        return
      rospy.sleep(0.01)
      if rospy.is_shutdown():
        return
    self.rate = criros.utils.read_parameter('{0}joint_state_controller/publish_rate'.format(self.ns), 125)
    self._num_joints = len(self._joint_names)
    rospy.logdebug('Topic [%sjoint_states] found' % self.ns)
  def joint_states_cb(self,msg):
    actual_joint_names = ['j1','j2','j3','j4','j5','j6']
    position = []
    effort = []
    name = []
    for joint_name in actual_joint_names:
      if joint_name in msg.name:
        idx = msg.name.index(joint_name)
        name.append(msg.name[idx])
        effort.append(msg.effort[idx])
        position.append(msg.position[idx])
    if set(name) == set(actual_joint_names):
      self._current_jnt_positions = np.array(position)
      self._current_jnt_efforts = np.array(effort)
      self._joint_names = list(name)
  def get_joint_position(self):
    return np.array(self._current_jnt_positions)
    
#JointController Class: give the position of joints to Gazebo
class JointController(ReadJointPosition):
  def __init__(self,namespace = '',timeout = 5.0):
    super(JointController, self).__init__(namespace, timeout=timeout)
    if not hasattr(self, '_joint_names'):
      raise rospy.ROSException('JointPositionController timed out waiting joint_states topic: {0}'.format(namespace))
    self._cmd_pub = dict()
    for joint in self._joint_names:
      self._cmd_pub[joint] = rospy.Publisher('%s%s/command' % (self.ns, joint), Float64, queue_size=3)
    # Wait for the joint position controllers
    controller_list_srv = self.ns + 'controller_manager/list_controllers'
    rospy.logdebug('Waiting for the joint position controllers...')
    rospy.wait_for_service(controller_list_srv, timeout=timeout)
    list_controllers = rospy.ServiceProxy(controller_list_srv, ListControllers)
    expected_controllers = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6')
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
      if (rospy.get_time() - start_time) > timeout:
        raise rospy.ROSException('JointPositionController timed out waiting for the controller_manager: {0}'.format(namespace))
      rospy.sleep(0.01)
      found = 0
      try:
        res = list_controllers()
        for state in res.controller:
          if state.name in expected_controllers:
            found += 1
      except:
        pass
      if found == len(expected_controllers):
        break
    rospy.loginfo('JointPositionController initialized. ns: {0}'.format(namespace))
  
  def set_joint_positions(self,jnt_positions):
    if not self.valid_jnt_command(jnt_positions):
      rospy.logwarn('A valid joint positions command should have %d elements' % (self._num_joints))
      return
    # Publish the point for each joint
    for name, q in zip(self._joint_names, jnt_positions):
      try:
        self._cmd_pub[name].publish(q)
      except:
        pass
        
  def valid_jnt_command(self, command):
    return ( len(command) == self._num_joints )

#FT sensor class: providing Wrench values from ft sensor with low pass filter    
class FTsensor(object):
  queue_len = 10
  def __init__(self, namespace='', timeout=3.0):
    ns = criros.utils.solve_namespace(namespace)
    self.rate = 250
    self.wrench_rate = 250
    self.wrench_filter = criros.filters.ButterLowPass(2.5, self.rate, 2)
    self.wrench_window = int(self.wrench_rate)
    assert( self.wrench_window >= 5)
    self.wrench_queue = collections.deque(maxlen=self.wrench_window)
    rospy.Subscriber('%sft_sensor/diagnostics' % ns, DiagnosticArray, self.cb_diagnostics)
    rospy.Subscriber('%sft_sensor/raw' % ns, WrenchStamped, self.cb_raw)
    initime = rospy.get_time()
    while not rospy.is_shutdown() and not self.is_raw_alive():
      rospy.sleep(0.1)
      if (rospy.get_time() - initime) > timeout:
        rospy.logwarn('FTSensor: Cannot read raw wrench')
        return
    rospy.loginfo('FTSensor successfully initialized')
  def add_wrench_observation(self,wrench):
    self.wrench_queue.append(np.array(wrench))
  def cb_diagnostics(self, msg):
    self.diagnostics_msg = msg
  def cb_raw(self, msg):
    self.raw_msg = copy.deepcopy(msg)
    self.add_wrench_observation(criros.conversions.from_wrench(self.raw_msg.wrench))
  def is_raw_alive(self):
    diagnostics = hasattr(self, 'diagnostics_msg')
    raw = hasattr(self, 'raw_msg')
    return (diagnostics and raw)
  def get_raw_wrench(self):
    if len(self.wrench_queue) < self.wrench_window:
      return None    
    wrench_filtered = self.wrench_filter(np.array(self.wrench_queue))
    return wrench_filtered[-1,:]
class Insertion_Gazebo(object):
  def __init__(self):
    self.denso_position_controller = JointController('denso') #create an instance of joint controller
    self.ft_sensor = FTsensor('denso')                      # create an instance of FT sensor
    self.gripper_controller = controller.Robotiq('denso')   #activate robotiq gripper
    self.js_rate = criros.utils.read_parameter('/denso/joint_state_controller/publish_rate', 250.0) #read publish rate if it does exist, otherwise set publish rate
    self.rate = rospy.Rate(self.js_rate)
    T = 1. / self.js_rate
    # setting up Openrave environment
    world_file = 'catkin_ws/src/intro-osr/envs/force_control_demo.xml'
    self.env = orpy.Environment()
    self.env.Load(world_file)
    self.env.SetViewer('qtcoin')
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.SetActiveManipulator('denso_robotiq_85_gripper')
    idle = self.manip.GetEndEffectorTransform()
    self.robot.SetActiveDOFs(range(6))
    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype = orpy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
      ikmodel.autogenerate()
    lmodel = orpy.databases.linkstatistics.LinkStatisticsModel(self.robot)
    if not lmodel.load():
      lmodel.autogenerate()
    lmodel.setRobotResolutions(0.001)
    lmodel.setRobotWeights()
    taskmanip = orpy.interfaces.TaskManipulation(self.robot)
    basemanip = orpy.interfaces.BaseManipulation(self.robot)
    
    #move the hand to position preparing for picking up the pin
    pin = self.env.GetKinBody('pin')
    tr_pin = pin.GetTransform()
    Pgoal = tr_pin[:3,3] + np.array([0,0,0.017])
    found = False
    Tgoal = tr.compose_matrix(angles = [0, -np.pi,np.pi/2], translate = Pgoal)
    traj = basemanip.MoveToHandPosition(matrices = [Tgoal], seedik = 10, execute = False, outputtrajobj = True)
    self.robot.WaitForController(0)
    traj_spec = traj.GetConfigurationSpecification()
    traj_duration = traj.GetDuration()
    step_num = traj_duration // T
      
    #execute trajectory in gazebo
    for t in np.append(np.arange(0, traj_duration, T), traj_duration):
      self.denso_position_controller.set_joint_positions(list(traj_spec.ExtractJointValues(traj.Sample(t), self.robot, self.manip.GetArmIndices())))
      self.robot.SetDOFValues(list(self.denso_position_controller.get_joint_position()),self.manip.GetArmIndices())
      self.rate.sleep()
      
    #close the gripper in Openrave
    taskmanip.CloseFingers()
    self.robot.WaitForController(0)
    self.robot.Grab(pin)
    
    #close the gripper in Gazebo
    self.gripper_controller.command(0.018, 0.013, 80)
    self.gripper_controller.wait()
    time.sleep(20) #need to wait for around 20s to create joints between gripper and object, otherwise the object will fall down as the robot arm moves
    
    #move pin to the position, prepare for moving down until contact
    tr_hole = np.array([0.4,0, 0.25])
    Thole = tr.compose_matrix(angles = [0,-np.pi,np.pi/2],translate = tr_hole)
    traj = basemanip.MoveToHandPosition(matrices = [Thole] ,seedik = 10, execute = False, outputtrajobj = True)
    self.robot.WaitForController(0)
    traj_spec = traj.GetConfigurationSpecification()
    traj_duration = traj.GetDuration()
    step_num = traj_duration // T
    for t in np.append(np.arange(0,traj_duration,T),traj_duration):
      self.denso_position_controller.set_joint_positions(list(traj_spec.ExtractJointValues(traj.Sample(t),self.robot, self.manip.GetArmIndices())))
      self.robot.SetDOFValues(list(self.denso_position_controller.get_joint_position()),self.manip.GetArmIndices())
      self.rate.sleep()
      
    rospy.sleep(1.0)
    
    # moving down util contact, prepare for sliding the pin on the surface
    dt = 1. / self.js_rate
    Kf = 10000.
    Kp = np.array([1., 1., 1.]) * 1. 
    Kv = np.array([1., 1., 1.]) * 40     
    Fr = np.array([0., 0., -15])
    qc = self.denso_position_controller.get_joint_position()
    Fe_prev = np.zeros(3)
    xf = np.zeros(3)
    dxf = np.zeros(3)
    rospy.loginfo('Going down')
    self.wrench_offset = self.ft_sensor.get_raw_wrench()
    while not rospy.is_shutdown():
      q_actual = self.denso_position_controller.get_joint_position()
      self.robot.SetDOFValues(q_actual, self.manip.GetArmIndices())
      We = self.ft_sensor.get_raw_wrench() - self.wrench_offset
      bTe = self.manip.GetEndEffectorTransform()
      bXeF = criros.spalg.force_frame_transform(bTe)
      Wb = np.dot(bXeF, We)
      Fb = -Wb[:3]
      Fr[0:2] = Fb[0:2]
      if np.linalg.norm(Fb) >= np.linalg.norm(Fr):
        rospy.loginfo('Surface contacted, preparing for sliding')
        break
      # Force PD compensator
      Fe = (Fr - Fb) / Kf
      dFe = (Fe - Fe_prev)
      Fe_prev = Fe
      dxf = (Kp*Fe + Kv*dFe) * dt
      xf += dxf
      # Velocity-based operational space controller
      J = self.manip.CalculateJacobian()
      dqc = np.dot(np.linalg.pinv(J), dxf)
      qc += dqc
      self.denso_position_controller.set_joint_positions(qc)
      self.rate.sleep()
      # Safety limits: displacement and max force
      if np.linalg.norm(xf) > 0.05:
        rospy.loginfo('Maximum displacement exceeded')
        break
      if np.linalg.norm(Fb) >= 20.:
        rospy.loginfo('Maximum force exceeded')
        break
    

