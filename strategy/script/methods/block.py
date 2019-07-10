#!/usr/bin/env python
from __future__ import print_function
import rospy
import math

class Block(object):

  def ClassicBlocking(self, goal_dis, goal_ang ,ball_dis ,ball_ang,goal_right,goal_left,imu_ang):
    imu_ang_deg=math.degrees(imu_ang)
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass

    v_x   = (ball_dis * math.sin(math.radians(ball_ang))) * math.sin(math.radians(imu_ang_deg))
    v_y   = (ball_dis * math.sin(math.radians(ball_ang))) * math.cos(math.radians(imu_ang_deg))
    v_yaw = -imu_ang_deg

    return v_x, v_y, v_yaw
