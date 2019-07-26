#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot
from blocker import Core

SOCCER_BALL_RADIUS = 0

class Block(object,Core):
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

  def ClassicRlimit(self,ball_dis ,ball_ang,goal_dis,goal_ang,imu_ang):
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

  def NoBallReturning(self, goal_dis, goal_ang):
    
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = 0
    return v_x, v_y, v_yaw
  
  def GuardPenalting(self, ball_dis, ball_ang,pwm_x,pwm_y):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = (ball_dis * math.cos(math.radians(ball_ang)))-pwm_x
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))+pwm_y
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def ClassicReturning(self, ball_ang,goal_dis, goal_ang):
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def ClassicPushung(self, ball_dis, ball_ang,pwm_x,pwm_y):
    ball_dis = ball_dis - SOCCER_BALL_RADIUS
    v_x   = (ball_dis * math.cos(math.radians(ball_ang)))-pwm_x
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))+pwm_y
    v_yaw = ball_ang
    return v_x, v_y, v_yaw

  def ClassicWaiting(self,ball_dis ,ball_ang,goal_dis,goal_ang):
    v_x   = 0
    v_y   = 0
    v_yaw = 0

    return v_x, v_y, v_yaw

  def NoBallWaiting(self,goal_dis,goal_ang):
    v_x   = 0
    v_y   = 0
    v_yaw = 0

    return v_x, v_y, v_yaw

  


