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

  def Rotate(self, x, y, theta):
    _x = x*math.cos(math.radians(theta)) - y*math.sin(math.radians(theta))
    _y = x*math.sin(math.radians(theta)) + y*math.cos(math.radians(theta))
    return _x, _y
  def BlockCheck(self, goal_dis, goal_ang, our_side):
    robot_info = self.GetRobotInfo()
    obstacles_info = self.GetObstacleInfo()
    obs = obstacles_info["detect_obstacles"]
    # obs_filter = self.obstacle_fileter(obs, robot_info)
    near_robot = self.GetState("near_robot")
    obs_filter = self.obstacle_fileter(obs, robot_info, near_robot)

    tmp = 0
    yaw = 0
    if(our_side=="Yellow"):
      tmp = -1
      yaw = 0
    elif(our_side=="Blue"):
      tmp = 1
      yaw = 180

    block_x = 200
    block_y = 160
    min_dis = 999
    obs_dis = 0 
    obs_ang = 0

    block_flag = False
    for i in range (0, len(obs_filter), 4):
        dis = obs_filter[i+0]
        ang = obs_filter[i+1]

        rox = dis * math.cos(math.radians(ang))
        roy = dis * math.sin(math.radians(ang))
        rrox, rroy = self.Rotate(rox, roy, robot_info['location']['yaw'])
        gox = rrox + robot_info['location']['x']
        goy = rroy + robot_info['location']['y']
        grx = robot_info['location']['x']
        gry = robot_info['location']['y']
        or_dis = math.sqrt(math.pow(gox-grx,2)+math.pow(goy-gry,2))
        if(abs(grx)>abs(gox)):
          if(our_side=="Yellow"):
            if(gox<0 and gox>block_x*tmp and abs(goy)<block_y):
                if(min_dis> or_dis):
                  min_dis = or_dis
                  obs_dis = dis
                  obs_ang = ang
                  block_flag = True
          elif(our_side=="Blue"):
            if(gox>0 and gox<block_x*tmp and abs(goy)<block_y):
                if(min_dis> or_dis):
                  min_dis = or_dis
                  obs_dis = dis
                  obs_ang = ang
                  block_flag = True
    return block_flag, obs_dis, obs_ang
  def ClassicDefense(self, goal_dis, goal_ang, our_side, near_robot=None): #back to our side
    robot_info = self.GetRobotInfo()

    tmp = 0
    yaw = 0
    if(our_side=="Yellow"):
      tmp = -1
      yaw = 0
    elif(our_side=="Blue"):
      tmp = 1
      yaw = 180

    p_x = 0
    p_y = 0
    p_yaw = 0
    #========back to defense====
    r_x=999
    r_y=999
    if(near_robot is not None):
      r_x = near_robot['position']['x']
      r_y = near_robot['position']['y']
    # print(r_x, r_y)
    back_y_dis = 80
    dis_tmp = []
    dis_tmp.append(math.hypot(robot_info['location']['x'] - 150*tmp, robot_info['location']['y'] - (back_y_dis)))
    dis_tmp.append(math.hypot(robot_info['location']['x'] - 150*tmp, robot_info['location']['y'] - (back_y_dis*(-1))))
    dis_tmp.append(math.hypot(r_x - 150*tmp, r_y - (back_y_dis)))
    dis_tmp.append(math.hypot(r_x - 150*tmp, r_y - (back_y_dis*(-1))))
    self_pos=None
    teammate_pos=None
    min_dis = 999
    min_dis_count = 0
    for i in range (0, len(dis_tmp), 1):
      # print("dis_tmp[i]", dis_tmp[i])
      if(min_dis>dis_tmp[i]):
        min_dis = dis_tmp[i]
        min_dis_count = i
    if(min_dis_count==0):
      self_pos = "up"
      teammate_pos = "down"
    elif(min_dis_count==1):
      self_pos = "down"
      teammate_pos = "up"
    elif(min_dis_count==2):
      teammate_pos = "up"
      self_pos = "down"
    elif(min_dis_count==3):
      teammate_pos = "down"
      self_pos = "up"
    # print("min_dis_count", min_dis_count)
    # print("self_pos", self_pos)
    if(self_pos=="down"):
      p_x = 150*tmp
      p_y = back_y_dis*(-1)
      p_yaw = 0+yaw
    else:
      p_x = 150*tmp
      p_y =  back_y_dis
      p_yaw = 0+yaw

    return p_x, p_y, p_yaw
  
