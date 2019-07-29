#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot


SOCCER_BALL_RADIUS = 0

class Block(Robot):

  cp_value = 0

  def ClassicBlocking(self, ball_dis, ball_ang, front_ang, pwm_y):
    o_x   = 0
    o_y   = ball_dis * math.sin(math.radians(ball_ang))
    v_x, v_y = self.Rotate(o_x, o_y, front_ang) 
    v_y = v_y + pwm_y
    v_yaw = self.cp_value - front_ang
    return v_x, v_y, v_yaw

  def GuardPenalting(self, ball_dis, ball_ang, pwm_x, pwm_y):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = (ball_dis * math.cos(math.radians(ball_ang)))-pwm_x
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))+pwm_y
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def Return(self,goal_dis, goal_ang, front_ang):
    distance = 70
    v_x   = distance - abs(goal_dis * math.cos(math.radians(goal_ang)))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = self.cp_value-front_ang
    return v_x, v_y, v_yaw

  def ClassicPushing(self, ball_dis, ball_ang, front_ang):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    o_x   = (ball_dis * math.cos(math.radians(ball_ang)))
    o_y   = (ball_dis * math.sin(math.radians(ball_ang)))
    v_x, v_y = self.Rotate(o_x, o_y, front_ang)
    v_yaw = ball_ang
    return v_x, v_y, v_yaw



  


