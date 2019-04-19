#!/usr/bin/env python3
import rospy
import math

class Attack(object):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None} 

  def ClassicAttacking(self, goal_dis, goal_ang):
    self.__pub_info['v_x']   = goal_dis * math.cos(math.radians(goal_ang))
    self.__pub_info['v_y']   = goal_dis * math.sin(math.radians(goal_ang))
    self.__pub_info['v_yaw'] = goal_ang

    return self.__pub_info