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
    robot_info = self.GetRobotInfo()
   
    v_x   = tx - robot_info['location']['x']
    v_y   = ty - robot_info['location']['y']
    o_x, o_y = self.Rotate(v_x, v_y, robot_info['location']['yaw'] * -1)

    v_yaw = tyaw - robot_info['location']['yaw']
    if abs(v_yaw - 360) < abs(v_yaw):
      o_yaw = v_yaw - 360
    elif abs(v_yaw + 360) < abs(v_yaw):
      o_yaw = v_yaw + 360
    else:
      o_yaw = v_yaw

    remaining = math.sqrt(o_x**2 + o_y**2)

    return o_x, o_y, o_yaw, remaining