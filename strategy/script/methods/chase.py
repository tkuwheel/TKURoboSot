#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np

class Chase(object):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None}
  __goal = None

  def ClassicRounding(self, goal_ang, ball_dis, ball_ang):
    alpha = math.radians(ball_ang - goal_ang)
    beta = 0.7
    if abs(alpha) > beta:
      if alpha > 0 :
        alpha = beta
      else:
        alpha = -beta
    else:
      pass

    br_x = ball_dis * math.cos(math.radians(ball_ang))
    br_y = ball_dis * math.sin(math.radians(ball_ang))
    self.__pub_info['v_x']   = br_x * math.cos(alpha) - br_y * math.sin(alpha)
    self.__pub_info['v_y']   = br_x * math.sin(alpha) + br_y * math.cos(alpha)
    self.__pub_info['v_yaw'] = goal_ang

    return self.__pub_info

  def StraightForward(self, ball_dis, ball_ang):
    self.__pub_info['v_x']   = ball_dis * math.cos(math.radians(ball_ang))
    self.__pub_info['v_y']   = ball_dis * math.sin(math.radians(ball_ang))
    self.__pub_info['v_yaw'] = ball_ang
    return self.__pub_info

  def Orbit(self, goal_ang, velocity = 50):
    orbit_radius = 33.5 # 22.5 + 11 cm
    w = velocity / orbit_radius

    self.__pub_info['v_x']   = 0
    self.__pub_info['v_y']   = velocity
    self.__pub_info['v_yaw'] = w * np.sign(velocity) * -1
    return self.__pub_info