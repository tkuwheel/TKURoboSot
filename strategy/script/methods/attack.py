#!/usr/bin/env python
from __future__ import print_function
import rospy
import math

class Attack(object):

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


  def block_attack(self, goal_dis, goal_ang):
    

    go_x = goal_dis * math.cos(math.radians(goal_ang))
    go_y = goal_dis * math.sin(math.radians(goal_ang))

    v_x   = go_x * math.cos(math.radians(100)) - go_y * math.sin(math.radians(100))
    v_y   = go_y * math.sin(math.radians(100)) + go_x * math.cos(math.radians(100))
    v_yaw = goal_ang
    
    return v_x, v_y, v_yaw