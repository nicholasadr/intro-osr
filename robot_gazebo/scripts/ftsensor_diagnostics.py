#!/usr/bin/env python
import os
import rospy
import criros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  ns = rospy.get_namespace()
  prefix = ns[1:] if ns[0] == '/' else ns
  publish_rate = criros.utils.read_parameter('~publish_rate', 1.0)
  pub = rospy.Publisher('ft_sensor/diagnostics', DiagnosticArray, queue_size=3)
  msg = DiagnosticArray()
  msg.header.frame_id = '{0}base_link'.format(prefix)
  status = DiagnosticStatus()
  status.level = DiagnosticStatus.OK
  status.name = 'NetFT RDT Driver'
  status.message = 'OK'
  status.hardware_id = 'ATI Gamma'
  status.values.append(KeyValue('IP Address', '192.168.0.12'))
  msg.status.append(status)
  rate = rospy.Rate(publish_rate)
  while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
