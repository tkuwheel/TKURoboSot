#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot


SOCCER_BALL_RADIUS = 0

class Block(Robot):

  def __init__(self):
    pass

  def ClassicBlocking(self, ball_dis, ball_ang, front_ang, pwm_x, pwm_y, cp_value):
    t = self.GetRobotInfo()
    o_x   = - pwm_x
    o_y   = ball_dis * math.sin(math.radians(ball_ang)) + pwm_y
    # print("cp_value", cp_value)
    # v_x, v_y = self.Rotate(o_x, o_y, front_ang) 
    v_yaw = cp_value - front_ang

    v_x=o_x
    v_y=o_y
    v_yaw = 0
    if v_x < 0 :
      v_x = 0
    return v_x, v_y, v_yaw

  def GuardPenalting(self, ball_dis, ball_ang, pwm_x, pwm_y):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = (ball_dis * math.cos(math.radians(ball_ang)))-pwm_x
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))+pwm_y
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def Return(self,goal_dis, goal_ang, front_ang, cp_value):
    # distance = 70
    # # o_x   = distance * math.cos(math.radians(goal_ang))
    # o_x   = distance - abs(goal_dis * math.cos(math.radians(goal_ang)))
    # o_y   = goal_dis * math.sin(math.radians(goal_ang))
    # print("cp_value", cp_value, "position['location']['yaw']", front_ang)
    # v_yaw = cp_value - front_ang
    # v_x, v_y = self.Rotate(o_x, o_y, front_ang)
    # return v_x, v_y, v_yaw
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = cp_value-front_ang
    if abs(v_yaw - 360) < abs(v_yaw):
      v_yaw = v_yaw - 360
    elif abs(v_yaw + 360) < abs(v_yaw):
      v_yaw = v_yaw + 360
    # print("cp_value", cp_value, "position['location']['yaw']", front_ang)w
    return v_x, v_y, v_yaw


  def ClassicPushing(self, ball_dis, ball_ang, front_ang):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    o_x   = (ball_dis * math.cos(math.radians(ball_ang)))
    o_y   = (ball_dis * math.sin(math.radians(ball_ang)))
    v_x, v_y = self.Rotate(o_x, o_y, front_ang)
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  



  


