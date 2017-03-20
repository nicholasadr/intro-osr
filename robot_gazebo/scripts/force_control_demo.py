#! /usr/bin/env python
from set_up import *
def run_demo():
  IG = Insertion_Gazebo()
  """
  Insertion_Gazebo class has the following attributes:
  self.denso_position_controller            self.js_rate
  self.ft_sensor                            self.rate
  self.gripper_controller                   self.env
  self.robot                                self.manip
  self.wrench_offset
  """
  #slide the pin on the table surface
  vlin = 0.003         #linear velocity
  dr_slide = [0.,1.]   # sliding direction on x-y plane
  dt = 1./ IG.js_rate  # time step  
  x0,y0,z0 = IG.manip.GetEndEffectorTransform()[:3,3] #position of gripper at the starting point
  xr = [x0,y0,z0]
  timeout = 20         
  t = 0
  Kp_pos = np.array([1, 1, 0])          #proportion controller of position
  Kv_pos = np.array([0.05, 0.05, 0])    #derivative controller of position
  Kp_force = np.array([0, 0, 3.4e-5])   #proportion controller of force
  Kv_force = np.array([0, 0, 8e-6])     #derivative controller of force
  bTe = IG.manip.GetEndEffectorTransform()
  bXeF = criros.spalg.force_frame_transform(bTe) # transformation matrix that convert wrench wrt body frame to  wrench in space frame
  We = IG.ft_sensor.get_raw_wrench() - IG.wrench_offset
  Wb = np.dot(bXeF, We)                 #wrench (the environment inserts on the end effector) is represented in space frame
  Fe_prev = -Wb[:3]                     #extract force vector only, negative sign indicates this is the force inserted on the environment by the robot
  xe_prev = [x0, y0, z0]
  qc = IG.denso_position_controller.get_joint_position()
  force_data = list()     #data for graph
  time_data = list()
  initime = rospy.get_time()
  while not rospy.is_shutdown() and (rospy.get_time() - initime) < timeout:
    t += dt
    if t < timeout/2.0:
      xr[0] += dr_slide[0] * vlin * dt
      xr[1] += dr_slide[1] * vlin * dt
    else:
      xr[0] -= dr_slide[0] * vlin * dt
      xr[1] -= dr_slide[1] * vlin * dt

    q_actual = IG.denso_position_controller.get_joint_position()
    IG.robot.SetDOFValues(q_actual, IG.manip.GetArmIndices())
    # Transform wrench to the base_link frame
    We = IG.ft_sensor.get_raw_wrench() - IG.wrench_offset
    bTe = IG.manip.GetEndEffectorTransform()
    bXeF = criros.spalg.force_frame_transform(bTe)
    Wb = np.dot(bXeF, We)
    Fb = -Wb[:3]
    Fr = np.array([0., 0., -15])
    Fr[:2] = Fb[:2]
    Fe = Fr - Fb
    dFe = (Fe - Fe_prev) / dt
    Fe_prev = Fe
    dxf_force = (Kp_force*Fe + Kv_force*dFe) * dt
    xb = IG.manip.GetEndEffectorTransform()[:3, 3]
    xe = xr - xb
    dxe = (xe - xe_prev)
    xe_prev = xe
    dxf_pos = (Kp_pos*xe + Kv_pos*dxe) * dt
    dxf = dxf_force + dxf_pos
    J = IG.manip.CalculateJacobian()
    dqc = np.dot(np.linalg.pinv(J), dxf)
    qc += dqc
    force_data.append(Fb[2])
    time_data.append(rospy.get_time() - initime)
    IG.denso_position_controller.set_joint_positions(qc)
    IG.rate.sleep()
  rospy.loginfo("Complete hybrid force control demo")
  plt.plot(time_data,force_data, label = "Controlled force in Z direction")
  plt.axhline(y=-15.0, xmin=0, xmax=20., linewidth=2, color = 'r',label = "reference line")
  plt.axis([0,20,-20,0])
  plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
         ncol=2, mode="expand", borderaxespad=0.)
  plt.show()    
    

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  run_demo()
  rospy.loginfo('Shuting down [%s] node' % node_name)
