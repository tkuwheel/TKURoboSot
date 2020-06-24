#!/usr/bin/env python
import rospy
import sys
import math
import time
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client

class Strategy(object):
  def __init__(self, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(200)
    self.robot = MyStateMachine(sim)
    self.main()

  def main(self):
    while not rospy.is_shutdown():
      self.robot.PubCurrentState()
      # print(self.robot.current_state)

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()

      # Can not find ball when starting
      if targets is None or targets['ball']['ang'] == 999 and self.robot.game_start:
        # print("Can not find ball")
        # self.robot.toIdle()
        self.robot.toFormation()
      else:
        if not self.robot.is_idle and not self.robot.game_start:
          self.robot.toIdle()

        if self.robot.is_idle:
          if self.robot.game_start:
            # self.robot.toChase()
            self.robot.toFormation()
            
        if self.robot.is_chase:
          if self.robot.CheckBallHandle():
            self.robot.toAttack()

        if self.robot.is_attack:
          if not self.robot.CheckBallHandle():
            self.robot.toChase()
          elif  abs(targets[self.robot.opp_side]['ang']) < self.robot.atk_shoot_ang and \
                abs(targets[self.robot.opp_side]['dis']) < self.robot.atk_shoot_dis:
            self.robot.toShoot(100)

        if self.robot.is_shoot:
          self.robot.toAttack()

        if rospy.is_shutdown():
          break

        ## Keep Current State Running
        keepState = 'to' + self.robot.current_state.name
        # print(keepState)
        if(keepState!='toIdle'):
          getattr(self.robot, keepState)()

        self.rate.sleep()

if __name__ == '__main__':
  try:
      # s = Strategy(True) # True is simulated mode
      s = Strategy(False)
  except rospy.ROSInterruptException:
    pass 
