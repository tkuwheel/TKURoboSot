#!/usr/bin/env python
from __future__ import print_function
import rospy
import math

class Block(object):

  cp_value = 0

  def ClassicBlocking(self, goal_dis, goal_ang ,ball_dis ,ball_ang,goal_right,goal_left,imu_ang,ball_speed_y):
    imu_ang = imu_ang * -1
    imu_ang_deg=math.degrees(imu_ang)
    imu_ang_deg = (imu_ang_deg + 360)%360
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass

    v_x   = 0
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))+ball_speed_y
    o_x   = (-v_y*(math.sin(math.radians(-imu_ang_deg)))+v_x*(math.cos(math.radians(-imu_ang_deg))))
    o_y   = (v_y*(math.cos(math.radians(-imu_ang_deg)))+v_x*(math.sin(math.radians(-imu_ang_deg))))
    v_yaw = self.cp_value - imu_ang_deg
    return o_x, o_y, v_yaw
