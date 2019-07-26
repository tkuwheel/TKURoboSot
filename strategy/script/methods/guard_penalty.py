#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np

SOCCER_BALL_RADIUS = 0

class Guard_Penalty(object): 
  def GuardPenalting(self, ball_dis, ball_ang,pwm_x,pwm_y):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = (ball_dis * math.cos(math.radians(ball_ang)))-pwm_x
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))+pwm_y
    v_yaw = ball_ang
    return v_x, v_y, v_yaw
