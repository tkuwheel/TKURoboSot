#!/usr/bin/env python
# encoding: utf-8
from __future__ import print_function
import rospy
import math
import numpy as np

SOCCER_BALL_RADIUS = 0

class Chase(object):
  def ClassicRounding(self, goal_ang, ball_dis, ball_ang):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
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
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = ball_dis * math.cos(math.radians(ball_ang))
    v_y   = ball_dis * math.sin(math.radians(ball_ang))
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def Run_point_relative(self, goal_dis, goal_ang, ball_dis, ball_ang):

    ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
    ball_y = ball_dis * math.sin(math.radians(ball_ang))

    door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
    door_y = goal_dis * math.sin(math.radians(goal_ang))

    defence_x   = (5* ball_x + door_x ) / 6					#防守位置
    defence_y   = (5* ball_y + door_y ) / 6
    defence_yaw = 0

    return defence_x , defence_y , defence_yaw


    
    
