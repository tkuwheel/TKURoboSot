#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot

class Behavior(Robot):
  def __init__(self):
    pass

  def Go2Point(self, tx, ty, tyaw):
    # print(self.robot.GetRobotInfo())
    get_the_point = 0
    robot_info = self.GetRobotInfo()
   
    v_x   = tx - robot_info['location']['x']
    v_y   = ty - robot_info['location']['y']
    v_yaw = tyaw - robot_info['location']['yaw']
    
    if abs(v_x) < 10 and abs(v_y) < 10 :
      get_the_point = 1

    return v_x, v_y, v_yaw, get_the_point