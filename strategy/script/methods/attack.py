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

#  def ClassicAttacking(self,obj):
#    __pub_info['v_x'] = obj['magenta_goal']['dis'] * math.cos(math.radians(obj['magenta_goal']['ang']))
#    __pub_info['v_y'] = obj['magenta_goal']['dis'] * math.sin(math.radians(obj['magenta_goal']['ang']))
#    __pub_info['v_yaw'] = obj['magenta_goal']['ang']
#    strategy_type = 'attack'
#
#    return __pub_info
'''def strategy(self,obj):
  __pub_info = {'v_x':0.0,'v_y':0.0,'v_yaw':0.0} 
  alpha = math.radians(obj['ball']['ang'] - obj['magenta_goal']['ang'])
  beta = 0.7
  if abs(alpha) > beta:
    angle_type = 'beta'
    if alpha > 0 :
      alpha = beta
    else:
      alpha = -beta
  else:
    angle_type = 'alpha'
  
 

  __pub_info['v_x'] = obj['magenta_goal']['dis'] * math.cos(math.radians(obj['magenta_goal']['ang']))
  __pub_info['v_y'] = obj['magenta_goal']['dis'] * math.sin(math.radians(obj['magenta_goal']['ang']))
  __pub_info['v_yaw'] = obj['magenta_goal']['ang']
  strategy_type = 'attack'


  
  return __pub_info'''
