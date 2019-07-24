#!/usr/bin/env python
import roslib
import rospy
import sys
import message_filters
import actionlib
import time
from std_msgs.msg import Bool 
from vision.msg import Object
from strategy.msg import RobotState
import strategy.msg
from strategy.cfg import RoleConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import dynamic_reconfigure.client

class RoleSelector(object):
  # create messages that are used to publish feedback/result
  _feedback = strategy.msg.PassingFeedback()
  _result   = strategy.msg.PassingResult()

  last_time = time.time()
  action_time = time.time()
  revival = True
  robot1 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0}
  robot2 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0}
  robot3 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0}
  r1_role = ""
  r2_role = ""
  r3_role = ""

  def __init__(self):
    self.rate = rospy.Rate(100)
    self.dserver = DynamicReconfigureServer(RoleConfig, self.Reconfigure)
    self.dclient = dynamic_reconfigure.client.Client("role_selector", timeout=30, config_callback=None)
    self._as = actionlib.SimpleActionServer("passing_action", strategy.msg.PassingAction, execute_cb=self.execute_cb)
    self._as.start()

    robot2_sub = message_filters.Subscriber('/robot2/strategy/state', RobotState)
    robot3_sub = message_filters.Subscriber('/robot3/strategy/state', RobotState)
    ts = message_filters.ApproximateTimeSynchronizer([robot2_sub, robot3_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(self.MulticastReceiver)

  def execute_cb(self, goal):
    success = True
    while not self.MyState()['ball_is_handled']:
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
      self._feedback.catcher_ball_dis = self.MyState()['ball_dis']
      self._as.publish_feedback(self._feedback)

    if success:
      self._result.catcher_res = True
      self._as.set_succeeded(self._result)

  @classmethod
  def PassingTo(self, catcher_ns):
    self._ac = actionlib.SimpleActionClient(catcher_ns + '/robot1/passing_action', strategy.msg.PassingAction)

  def MyState(self):
    if "robot1" in rospy.get_namespace():
      return self.robot1
    elif "robot2" in rospy.get_namespace():
      return self.robot2
    elif "robot3" in rospy.get_namespace():
      return self.robot3
    else:
      print("Wrong Namespace")

  def MyRole(self):
    if "robot1" in rospy.get_namespace():
      return self.r1_role
    elif "robot2" in rospy.get_namespace():
      return self.r2_role
    elif "robot3" in rospy.get_namespace():
      return self.r3_role
    else:
      print("Wrong Namespace")

  def MulticastReceiver(self, r2_data, r3_data):
    RoleSelector.last_time = time.time()
    RoleSelector.revival = True
    self.robot2['ball_is_handled'] = r2_data.ball_is_handled
    self.robot2['ball_dis']        = r2_data.ball_dis
    self.robot3['ball_is_handled'] = r3_data.ball_is_handled
    self.robot3['ball_dis']        = r3_data.ball_dis

  def Reconfigure(self, config, level):
    self.role    = config["role"]
    self.prority = config["prority"]
    return config

  def Supervisor(self):
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
      if time.time() - RoleSelector.last_time > 5:
        print("Lossing Connection with teammates......")
        time.sleep(1)
      else:
        if self.robot2['ball_is_handled']:
          self.r2_role = "Attacker"
          self.r3_role = "Supporter"
        elif self.robot3['ball_is_handled']:
          self.r2_role = "Supporter"
          self.r3_role = "Attacker"
        else:
          self.r2_role = "Attacker" if self.robot2['ball_dis'] < self.robot3['ball_dis'] else "Supporter"
          self.r3_role = "Supporter" if self.robot2 is "Attacker" else "Attacker"

      r.sleep()
    print("Good-bye")

if __name__ == '__main__':
  rospy.init_node('role_selector')
  try:
    ne = RoleSelector()
    ne.Supervisor()
  except rospy.ROSInterruptException: pass