#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
from robot.robot import Robot
from robot.obstacle import Obstacle

class Defense(Robot,Obstacle):
  def __init__(self):
    pass

  def ClassicDefense(self, goal_dis, goal_ang, our_side): #back to our side
    position = self.GetRobotInfo()
    #========back to defense====
    p_x = 0
    p_y = 0
    p_yaw = 0
    tmp = 0
    if(our_side=="Yellow"):
      tmp = -1
    elif(our_side=="Blue"):
      tmp = 1
    if(position['location']['y']<0):  
      p_x = 200*tmp
      p_y = -50
      p_yaw = 0
    else:
      p_x = 200*tmp
      p_y =  50
      p_yaw = 0
    
    return p_x, p_y, p_yaw
  
