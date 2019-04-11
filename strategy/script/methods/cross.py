#!/usr/bin/env python3
import rospy
import math

def rotate(self,obj):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None}              
  alpha = 0.5 #math.radians(obj['ball']['ang'])
  #br_x = obj['ball']['dis'] * math.cos(math.radians(obj['ball']['ang']))
  #br_y = obj['ball']['dis'] * math.sin(math.radians(obj['ball']['ang']))
  __pub_info['v_x'] =  math.cos(alpha) - math.sin(alpha)
  __pub_info['v_y'] =  math.sin(alpha)+ math.cos(alpha)
  __pub_info['v_yaw'] = obj['ball']['ang']
  strategy_type = 'cross'
  
  return __pub_info