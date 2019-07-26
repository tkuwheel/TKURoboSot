#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np



class NoBallRet(object): 
  def NoBallReturning(self, goal_dis, goal_ang):
    
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = 0
    return v_x, v_y, v_yaw
