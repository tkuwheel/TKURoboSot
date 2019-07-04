#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
from robot.robot import Robot
class Attack(Robot):
  def __init__(self):
    pass

  def ClassicAttacking(self, goal_dis, goal_ang):
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw

  def zoneAttacking(self, goal_dis, goal_ang):
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw


  def block_attack(self, t, side, run):
    robot_info = self.GetRobotInfo()
    shoot = 0
    v_x   = t[side]['dis'] * math.cos(math.radians(t[side]['ang']))
    v_y   = t[side]['dis'] * math.sin(math.radians(t[side]['ang']))
    v_yaw = t[side]['ang']
  
    if t[side]['dis'] <= 200:
      go_x = t[side]['dis'] * math.cos(math.radians(t[side]['ang']))
      go_y = t[side]['dis'] * math.sin(math.radians(t[side]['ang']))
      v_x   = go_x  * math.cos(math.radians(100)) - go_y  * math.sin(math.radians(100))
      v_y   = go_y  * math.sin(math.radians(100)) + go_x  * math.cos(math.radians(100)) 
     
      if side == 'Blue':
        if abs(robot_info['location']['yaw']) > run['point']['yaw'] :
          shoot = 1  
      else :
        if abs(robot_info['location']['yaw']) < run['point']['yaw'] :
          shoot = 1  
      

    return v_x, v_y, v_yaw, shoot