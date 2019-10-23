#!/usr/bin/env python

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

  def ClassicRounding2(self, goal_ang, ball_dis, ball_ang, ball_v_x, ball_v_y):
    prediction_ball_x = ball_dis * math.cos(math.radians(ball_ang)) + ball_v_x * 100 / 0.5 * 0.3 # m / 0.5s
    prediction_ball_y = ball_dis * math.sin(math.radians(ball_ang)) + ball_v_y * 100 / 0.5 * 0.3
    #ball_dis = math.sqrt(math.pow(prediction_ball_x,2)+math.pow(prediction_ball_y,2))
    #print("ball_ang", ball_ang, '!!!')
    ori_ball_ang = ball_ang
    if(abs(ball_v_y)> 2):
      ball_ang = ball_ang + 100 * ball_v_y /100
    #print("ball_v_x", ball_v_x)
    ball_dis_x = ball_dis
    #if(ball_dis < 50 and abs(ball_ang)>50 and ball_v_x < -10):
    #  ball_dis_x = ball_dis + ball_v_x * 100 / 0.5 * 0.3
    #if(ball_dis<40 and abs(ball_ang)<15 and ball_v_x>-1):
    #  ball_dis_x = ball_dis + 30

    #if(ball_v_x < -10):
    #  if(ball_ang > 0):
    #    ball_ang = ball_ang - 125 * ball_v_x / 100
    #  else:
    #    ball_ang = ball_ang + 125 * ball_v_x / 100
    #  print("fuck")
    #if(abs(ball_ang) < 30): 
      #ball_dis = math.sqrt(math.pow(prediction_ball_x,2)+math.pow(prediction_ball_y,2))
      #ball_ang = math.degrees(math.atan2(prediction_ball_y,prediction_ball_x))
    #print("ball_ang", ball_ang)
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    alpha = ball_ang - goal_ang

    if ball_dis > 100:
      beta = 15
    elif ball_dis > 50:
      beta = 60
    else:
      beta = 85

    if abs(alpha) > beta:
      alpha = beta * np.sign(alpha)
    else:
      pass

    #br_x = ball_dis * math.cos(math.radians(ball_ang))
    br_x = ball_dis_x * math.cos(math.radians(ball_ang))
    br_y = ball_dis * math.sin(math.radians(ball_ang))

    v_x   = br_x * math.cos(math.radians(alpha)) - br_y * math.sin(math.radians(alpha))
    v_y   = br_x * math.sin(math.radians(alpha)) + br_y * math.cos(math.radians(alpha))
    # if(abs(ball_ang)>40 and v_x<0 and v_x>-20):
    #   v_x = v_x-30
    # if(abs(ball_ang)>20 and v_x>0 and v_x<20):
    #   v_x = v_x+15
    # if(abs(v_x)>50):
    #   v_x=v_x*0.8
    # if(abs(v_y)>50):
    #   v_y=v_y*0.8
    v_yaw = goal_ang
    if(abs(ori_ball_ang - goal_ang)<40):
      v_yaw = ori_ball_ang
    return v_x, v_y, v_yaw

  def StraightForward(self, ball_dis, ball_ang):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = ball_dis * math.cos(math.radians(ball_ang))
    v_y   = ball_dis * math.sin(math.radians(ball_ang))
    v_yaw = ball_ang
    return v_x, v_y, v_yaw