#!/usr/bin/env python3
import rospy
import math

def strategy(self,obj):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None} 
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
  
 
  br_x = obj['ball']['dis'] * math.cos(math.radians(obj['ball']['ang']))
  br_y = obj['ball']['dis'] * math.sin(math.radians(obj['ball']['ang']))
  if obj['ball']['dis'] <= 50 and abs(obj['ball']['ang']) <= 20:
    __pub_info['v_x'] = obj['magenta_goal']['dis'] * math.cos(math.radians(obj['magenta_goal']['ang']))
    __pub_info['v_y'] = obj['magenta_goal']['dis'] * math.sin(math.radians(obj['magenta_goal']['ang']))
    __pub_info['v_yaw'] = obj['magenta_goal']['ang']
    strategy_type = 'attack'

  else:
    __pub_info['v_x'] = br_x * math.cos(alpha) - br_y * math.sin(alpha)
    __pub_info['v_y']= br_y * math.sin(alpha) + br_y * math.cos(alpha)
 
    __pub_info['v_yaw'] = obj['magenta_goal']['ang']
    strategy_type = 'chase'
  
  return __pub_info