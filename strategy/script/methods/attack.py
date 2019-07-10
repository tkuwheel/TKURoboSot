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


  def cross_over(self, t, side, run):
    robot_info = self.GetRobotInfo()
    shoot = 0

    go_x = t[side]['dis'] * math.cos(math.radians(t[side]['ang']))
    go_y = t[side]['dis'] * math.sin(math.radians(t[side]['ang']))
    
    v_x   = go_x  * math.cos(math.radians(run['yaw'])) - go_y  * math.sin(math.radians(run['yaw']))
    v_y   = go_x  * math.sin(math.radians(run['yaw'])) + go_y  * math.cos(math.radians(run['yaw'])) 

 
    if t[side]['dis'] > 250:
       
       v_yaw = t[side]['ang']
    
    else:
      if t[side]['ang'] > 0 :
        v_yaw = -80
      else :
        v_yaw = t[side]['ang']
   
    




    if t[side]['dis'] <= 200 and t[side]['ang']<=10:
      shoot = 1
            
    return v_x, v_y, v_yaw, shoot

  