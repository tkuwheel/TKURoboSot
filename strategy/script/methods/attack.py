#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
from robot.robot import Robot
from robot.obstacle import Obstacle

class Attack(Robot,Obstacle):
  def __init__(self):
    pass

  def ClassicAttacking(self, goal_dis, goal_ang):
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw

  def Cut(self, goal_dis, goal_ang, yaw):
    go_x = goal_dis * math.cos(math.radians(goal_ang))
    go_y = goal_dis * math.sin(math.radians(goal_ang))
    v_x   = go_x  * math.cos(math.radians(yaw)) - go_y  * math.sin(math.radians(yaw))
    v_y   = go_x  * math.sin(math.radians(yaw)) + go_y  * math.cos(math.radians(yaw))        
    v_yaw = goal_ang
    
    return v_x, v_y, v_yaw

  def Post_up(self, goal_dis, goal_ang,ranges, angle_increment):
    
    
    self.__goal_dis = goal_dis
    self.__goal_ang = goal_ang
    self.__ranges = ranges
    self.__angle_increment = angle_increment


    self.raw , object_dis= self.state(ranges) 
    self.edit = self.filter(self.raw)        
    obstacle_force_x , obstacle_force_y = self.Obstacle_segmentation(self.edit ,angle_increment , object_dis)
    
    if obstacle_force_x == 0 and obstacle_force_y == 0 :
        v_x   = goal_dis * math.cos(math.radians(goal_ang))
        v_y   = goal_dis * math.sin(math.radians(goal_ang))
        v_yaw = goal_ang

        return v_x , v_y , v_yaw

    else :
        v_x,v_y,v_yaw = self.Force_Calculation(obstacle_force_x , obstacle_force_y ,goal_ang, goal_dis,1)

    
    return v_x, v_y, v_yaw
  
  def Post_up2(self, goal_dis, goal_ang):
    robot_info = self.GetRobotInfo()
    obstacles_info = self.GetObstacleInfo()
    obs = obstacles_info["detect_obstacles"]
    #print(len(obs))
    if(len(obs) == 0):
      #print("NOOOO OBSTACLE!!!!")
      v_x   = goal_dis * math.cos(math.radians(goal_ang))
      v_y   = goal_dis * math.sin(math.radians(goal_ang))
      v_yaw = 0
    else:
      #print("YES OBSTACLE!!!!")
      obs_filter = self.obstacle_fileter(obs, robot_info)
      v_yaw = 0.7*self.obstacle_roate(obs_filter, robot_info)
      v_x, v_y = self.obstacle_escape(goal_dis, goal_ang, obs_filter, robot_info)

    return v_x, v_y, v_yaw
  
