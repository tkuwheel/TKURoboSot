#!/usr/bin/env python

import roslib
import rospy
import sys
import message_filters
import time
from std_msgs.msg import Bool 
from vision.msg import Object
from strategy.msg import RobotState
from strategy.cfg import RoleConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import dynamic_reconfigure.client

class RoleSelector():
  last_time = 0
  def __init__(self):
    self.rate = rospy.Rate(1000)
    self.dserver = DynamicReconfigureServer(RoleConfig, self.Reconfigure)
    self.dclient = dynamic_reconfigure.client.Client("role_selector", timeout=30, config_callback=None)

    robot2_sub = message_filters.Subscriber('/robot2/strategy/state', RobotState)
    robot3_sub = message_filters.Subscriber('/robot3/strategy/state', RobotState)
    ts = message_filters.ApproximateTimeSynchronizer([robot2_sub, robot3_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(self.MulticastReceiver)

    while not rospy.is_shutdown():
      if time.time() - RoleSelector.last_time > 10:
        print("Lossing Connection with teammates......")

      self.rate.sleep()

  def MulticastReceiver(self, r2_data, r3_data):
    RoleSelector.last_time = time.time()

  def Reconfigure(self, config, level):
    self.role    = config["role"]
    self.prority = config["prority"]
    return config

if __name__ == '__main__':
  rospy.init_node('role_selector')
  try:
    ne = RoleSelector()
  except rospy.ROSInterruptException: pass