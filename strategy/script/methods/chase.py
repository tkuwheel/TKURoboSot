#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np

class Chase(object):
  def ClassicRounding(self, goal_ang, ball_dis, ball_ang):
    alpha = ball_ang - goal_ang
    if ball_dis > 100:
      beta = 15
    elif ball_dis > 50:
      beta = 45
    else:
      beta = 70

    if abs(alpha) > beta:
      alpha = beta * np.sign(alpha)
    else:
      pass

    br_x = ball_dis * math.cos(math.radians(ball_ang))
    br_y = ball_dis * math.sin(math.radians(ball_ang))

    v_x   = br_x * math.cos(math.radians(alpha)) - br_y * math.sin(math.radians(alpha))
    v_y   = br_x * math.sin(math.radians(alpha)) + br_y * math.cos(math.radians(alpha))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw

  def StraightForward(self, ball_dis, ball_ang):
    v_x   = ball_dis * math.cos(math.radians(ball_ang))
    v_y   = ball_dis * math.sin(math.radians(ball_ang))
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def Orbit(self, goal_ang):
    orbit_radius = 33.5 # 22.5 + 11 cm
    velocity = goal_ang * -1.5
    w = velocity / orbit_radius


    v_x   = 0
    v_y   = velocity
    v_yaw = w * np.sign(velocity) * -1
    return v_x, v_y, v_yaw
