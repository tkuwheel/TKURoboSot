#!/usr/bin/env python3
import rospy
import math

class Chase(object):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None}

  def ClassicRounding(self, obj):
    alpha = math.radians(obj['ball']['ang'] - obj['magenta_goal']['ang'])
    beta = 0.7
    if abs(alpha) > beta:
      if alpha > 0 :
        alpha = beta
      else:
        alpha = -beta
    else:
      pass

    br_x = obj['ball']['dis'] * math.cos(math.radians(obj['ball']['ang']))
    br_y = obj['ball']['dis'] * math.sin(math.radians(obj['ball']['ang']))
    self.__pub_info['v_x']   = br_x * math.cos(alpha) - br_y * math.sin(alpha)
    self.__pub_info['v_y']   = br_x * math.sin(alpha) + br_y * math.cos(alpha)
    self.__pub_info['v_yaw'] = obj['magenta_goal']['ang']
    
    return self.__pub_info