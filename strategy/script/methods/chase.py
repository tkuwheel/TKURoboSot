#!/usr/bin/env python3
import rospy
import math

def strategy(self,obj):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None}              
  
  br_x = obj['ball']['dis'] * math.cos(math.radians(obj['ball']['ang']))
  br_y = obj['ball']['dis'] * math.sin(math.radians(obj['ball']['ang']))
  __pub_info['v_x'] = br_x #* math.cos(alpha) - br_y * math.sin(alpha)
  __pub_info['v_y']= br_y #* math.sin(alpha) + br_y * math.cos(alpha)

  __pub_info['v_yaw'] = obj['ball']['ang']
  strategy_type = 'chase'
  
  return __pub_info