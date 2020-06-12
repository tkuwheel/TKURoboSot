#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
def angle_adjustment(self, angle):
  imu_angle = angle
  imu_ang = imu_ang * -1
  imu_ang_deg=math.degrees(imu_ang)
  imu_ang_deg = (imu_ang_deg + 360)%360
  if imu_ang_deg>180:
    imu_ang_deg=imu_ang_deg-360
  else:
    pass
  return imu_angle
class Block(object):

  cp_value = 0
  correct_ang=[]
  def ClassicBlocking(self,ball_dis ,ball_ang,imu_ang,ball_speed_y):
    imu_ang = imu_ang * -1
    imu_ang_deg=math.degrees(imu_ang)
    imu_ang_deg = (imu_ang_deg + 360)%360
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass
    #if imu_ang_deg>10:
     # imu_ang_deg=5
    #elif imu_ang_deg<-10:
     # imu_ang_deg=-5
    #else:
     # pass
    self.correct_ang.append(imu_ang_deg)
    self.cp_value=self.correct_ang[0]
    v_x   = 0
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))
    o_x   = (-v_y*(math.sin(math.radians(-imu_ang_deg)))+v_x*(math.cos(math.radians(-imu_ang_deg))))
    o_y   = (v_y*(math.cos(math.radians(-imu_ang_deg)))+v_x*(math.sin(math.radians(-imu_ang_deg))))
    v_yaw = self.cp_value - imu_ang_deg
    return o_x, o_y, v_yaw

  def ClassicWaiting(self,imu_ang):
    imu_ang = imu_ang * -1
    imu_ang_deg=math.degrees(imu_ang)
    imu_ang_deg = (imu_ang_deg + 360)%360
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass
   # if imu_ang_deg>10:
    #  imu_ang_deg=5
    #elif imu_ang_deg<-10:
     # imu_ang_deg=-5
    #else:
     # pass

    self.correct_ang.append(imu_ang_deg)
    self.cp_value=self.correct_ang[0]
    v_x   = 0
    v_y   = 0
    v_yaw = self.cp_value-imu_ang_deg

    return v_x, v_y, v_yaw

  def ClassicReturning(self, ball_ang,goal_dis, goal_ang,imu_ang):
    imu_ang = imu_ang * -1
    imu_ang_deg=math.degrees(imu_ang)
    imu_ang_deg = (imu_ang_deg + 360)%360
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass
   # if imu_ang_deg>10:
    #  imu_ang_deg=5
   # elif imu_ang_deg<-10:
    #  imu_ang_deg=-5
   # else:
    #  pass
    self.correct_ang.append(imu_ang_deg)
    self.cp_value=self.correct_ang[0]
    distance = 75
    v_x   = distance-abs(goal_dis * math.cos(math.radians(goal_ang)))
    v_y   = 0#goal_dis * math.sin(math.radians(goal_ang))
    o_x   = (-v_y*(math.sin(math.radians(-imu_ang_deg)))+v_x*(math.cos(math.radians(-imu_ang_deg))))
    o_y   = (v_y*(math.cos(math.radians(-imu_ang_deg)))+v_x*(math.sin(math.radians(-imu_ang_deg))))
    v_yaw = self.cp_value-imu_ang_deg
    return o_x, o_y, v_yaw
    

  def ClassicPushung(self, ball_dis, ball_ang,pwm_x,pwm_y,imu_ang):
    imu_ang = imu_ang * -1
    imu_ang_deg=math.degrees(imu_ang)
    imu_ang_deg = (imu_ang_deg + 360)%360
    if imu_ang_deg>180:
      imu_ang_deg=imu_ang_deg-360
    else:
      pass
   # if imu_ang_deg>10:
    #  imu_ang_deg=5
   # elif imu_ang_deg<-10:
    #  imu_ang_deg=-5
   # else:
    #  pass
    self.correct_ang.append(imu_ang_deg)
    self.cp_value=self.correct_ang[0]
    v_x   = (ball_dis * math.cos(math.radians(ball_ang)))
    v_y   = (ball_dis * math.sin(math.radians(ball_ang)))
    v_yaw = ball_ang
    return v_x, v_y, v_yaw
