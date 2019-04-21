#!/usr/bin/env python3
import rospy
import math

'''<<<<<<< HEAD
def strategy(self,obj):
  __pub_info = {'v_x':0.0,'v_y':0.0,'v_yaw':0.0}              
  
  br_x = obj['ball']['dis'] * math.cos(math.radians(obj['ball']['ang']))
  br_y = obj['ball']['dis'] * math.sin(math.radians(obj['ball']['ang']))
  __pub_info['v_x'] = br_x #* math.cos(alpha) - br_y * math.sin(alpha)
  __pub_info['v_y']= br_y #* math.sin(alpha) + br_y * math.cos(alpha)

  __pub_info['v_yaw'] = obj['magenta_goal']['ang']
  strategy_type = 'chase'
  
  return __pub_info
======='''
class Chase(object):
  __pub_info = {'v_x':None,'v_y':None,'v_yaw':None}
  __goal = None

  def ClassicRounding(self, goal_ang, ball_dis, ball_ang):
    alpha = math.radians(ball_ang - goal_ang)
    beta = 0.7
    if abs(alpha) > beta:
      if alpha > 0 :
        alpha = beta
      else:
        alpha = -beta
    else:
      pass

    br_x = ball_dis * math.cos(math.radians(ball_ang))
    br_y = ball_dis * math.sin(math.radians(ball_ang))
    self.__pub_info['v_x']   = br_x * math.cos(alpha) - br_y * math.sin(alpha)
    self.__pub_info['v_y']   = br_x * math.sin(alpha) + br_y * math.cos(alpha)
    self.__pub_info['v_yaw'] = goal_ang
    
    return self.__pub_info

#  def ClassicRounding(self, goal_ang=None, ball_dis=None, ball_ang=None, obj):
#    alpha = math.radians(obj['ball']['ang'] - obj['magenta_goal']['ang'])
#    beta = 0.7
#    if abs(alpha) > beta:
#      if alpha > 0 :
#        alpha = beta
#      else:
#        alpha = -beta
#    else:
#      pass
#
#    br_x = obj['ball']['dis'] * math.cos(math.radians(obj['ball']['ang']))
#    br_y = obj['ball']['dis'] * math.sin(math.radians(obj['ball']['ang']))
#    self.__pub_info['v_x']   = br_x * math.cos(alpha) - br_y * math.sin(alpha)
#    self.__pub_info['v_y']   = br_x * math.sin(alpha) + br_y * math.cos(alpha)
#    self.__pub_info['v_yaw'] = obj['magenta_goal']['ang']
#    
#    return self.__pub_info
>>>>>>> tkurobot/master
