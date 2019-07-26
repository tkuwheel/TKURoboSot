#!/usr/bin/env python
from __future__ import print_function
import rospy
import math

class L_limit(object):

  def ClassicLlimit(self,ball_dis ,ball_ang,goal_dis,goal_ang,imu_ang):
    imu_ang = imu_ang * -1
    imu_ang_deg=math.degrees(imu_ang)
    imu_ang_deg = (imu_ang_deg + 360)%360
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass
    v_x   = 0
    v_y   = 0
    v_yaw = -imu_ang_deg

    return v_x, v_y, v_yaw
