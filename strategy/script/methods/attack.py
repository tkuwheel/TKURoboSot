#!/usr/bin/env python3
import rospy
import math

def strategy(self,obj):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None} 
  
  __pub_info['v_x'] = obj['magenta_goal']['dis'] * math.cos(math.radians(obj['magenta_goal']['ang']))
  __pub_info['v_y'] = obj['magenta_goal']['dis'] * math.sin(math.radians(obj['magenta_goal']['ang']))
  __pub_info['v_yaw'] = obj['magenta_goal']['ang']
  strategy_type = 'attack'
  
  return __pub_info