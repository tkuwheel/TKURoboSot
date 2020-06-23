#!/usr/bin/env python
# encoding: utf-8
from __future__ import print_function
import rospy
import math
import numpy as np
from robot.robot import Robot
from robot.obstacle import Obstacle

ORBIT_KP_V = -0.5
ORBIT_KP_W = 4.2

#REMAINING_RANGE_V = 20
#REMAINING_RANGE_YAW = 7

class Behavior(Robot,Obstacle):
  def __init__(self):
    self.penalty_angle = []

  def Orbit(self, goal_ang, REMAINING_RANGE_YAW = 5):
    orbit_radius = 33.5 # 22.5 + 11 cm
    velocity = goal_ang
    # velocity = velocity if abs(velocity) < 45 else 45 # maximum speed
    w = (velocity / orbit_radius)

    v_x   = 0
    v_y   = velocity * ORBIT_KP_V
    v_yaw = w * ORBIT_KP_W
    o_yaw = v_yaw if abs(v_yaw) > 0.2 else 0.2 * np.sign(v_yaw) # 0.2 is minimum speed

    remaining_yaw = o_yaw
    if abs(remaining_yaw) < REMAINING_RANGE_YAW:
      arrived = True
    else:
      arrived = False
    return v_x, v_y, o_yaw, arrived

  def Go2Point(self, tx, ty, tyaw, REMAINING_RANGE_V = 10, REMAINING_RANGE_YAW = 5):
    robot_info = self.GetRobotInfo()

    v_x   = tx - robot_info['location']['x']
    v_y   = ty - robot_info['location']['y']
    o_x, o_y = self.Rotate(v_x, v_y, robot_info['location']['yaw'] * -1)

    v_yaw = tyaw - robot_info['location']['yaw']
    if abs(v_yaw - 360) < abs(v_yaw):
      o_yaw = v_yaw - 360
    elif abs(v_yaw + 360) < abs(v_yaw):
      o_yaw = v_yaw + 360
    else:
      o_yaw = v_yaw

    remaining_v   = math.sqrt(o_x**2 + o_y**2)
    remaining_yaw = o_yaw
    if remaining_v < REMAINING_RANGE_V and abs(remaining_yaw) < REMAINING_RANGE_YAW:
      arrived = True
    else:
      arrived = False

    return o_x, o_y, o_yaw, arrived

  

  def relative_goal(self, goal_dis, goal_ang, ball_dis, ball_ang):

    if ball_dis < 100 and goal_dis > 150:
      ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
      ball_y = ball_dis * math.sin(math.radians(ball_ang))

      door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
      door_y = goal_dis * math.sin(math.radians(goal_ang))

      defence_x   = ( 10000 * ball_x + door_x ) / 2				        #防守位置
      defence_y   = ( 10000 * ball_y + door_y ) / 2
      defence_yaw = ball_ang
 
    elif ball_dis  < 150 and goal_dis < 150:
      ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
      ball_y = ball_dis * math.sin(math.radians(ball_ang))

      door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
      door_y = goal_dis * math.sin(math.radians(goal_ang))

      defence_x   = 0		                                		#avoid to go to the goal area
      defence_y   = 0
      defence_yaw = ball_ang
    else:
      ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
      ball_y = ball_dis * math.sin(math.radians(ball_ang))

      door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
      door_y = goal_dis * math.sin(math.radians(goal_ang))

      defence_x   = ( ball_x + door_x ) / 2				#防守位置
      defence_y   = ( ball_y + door_y ) / 2
      defence_yaw = ball_ang


    return defence_x , defence_y , defence_yaw

  def relative_ball(self, goal_dis, goal_ang, ball_dis, ball_ang):

    if ball_dis < 100 and goal_dis > 150:
      ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
      ball_y = ball_dis * math.sin(math.radians(ball_ang))

      door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
      door_y = goal_dis * math.sin(math.radians(goal_ang))

      defence_x   = (10000*ball_x + door_x ) / 10                              	#avoid to go to the goal area
      defence_y   = (10000*ball_y + door_y ) / 10
      defence_yaw = ball_ang

    elif ball_dis  < 150 and goal_dis < 150:

      ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
      ball_y = ball_dis * math.sin(math.radians(ball_ang))

      door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
      door_y = goal_dis * math.sin(math.radians(goal_ang))

      defence_x   = 0		                                		#avoid to go to the goal area
      defence_y   = 0
      defence_yaw = ball_ang
 
    else:

      ball_x = ball_dis * math.cos(math.radians(ball_ang))			#機器人看球的座標
      ball_y = ball_dis * math.sin(math.radians(ball_ang))

      door_x = goal_dis * math.cos(math.radians(goal_ang))			#機器人看門的座標
      door_y = goal_dis * math.sin(math.radians(goal_ang))

      defence_x   = (7.5*ball_x +door_x ) / 10               	#防守位置
      defence_y   = (7.5*ball_y +door_y ) / 10
      defence_yaw = ball_ang


    return defence_x , defence_y , defence_yaw

  def PenaltyTurning(self, side, run_yaw, dest_ang):
    robot_info = self.GetObjectInfo()
      
    position = self.GetRobotInfo()
    front_ang = math.degrees(position['imu_3d']['yaw'])
    v_yaw = front_ang - dest_ang
    if run_yaw == 0:
      v_yaw = robot_info[side]['ang']
    v_x = 0
    v_y = 0
    return v_x, v_y, v_yaw


  def Post_up(self, goal_dis, goal_ang,ranges, angle_increment):
    self.__goal_dis = goal_dis
    self.__goal_ang = goal_ang
    self.__ranges = ranges
    self.__angle_increment = angle_increment

    self.raw , object_dis= self.state(ranges) 
    self.edit = self.filter(self.raw)        
    obstacle_force_x , obstacle_force_y = self.Obstacle_segmentation(self.edit ,angle_increment , object_dis)
    
    if obstacle_force_x == 0 and obstacle_force_y == 0 :
        v_x   = goal_dis * math.cos(math.radians(goal_ang))
        v_y   = goal_dis * math.sin(math.radians(goal_ang))
        v_yaw = goal_ang

        return v_x , v_y , v_yaw
    else :
        v_x,v_y,v_yaw = self.Force_Calculation(obstacle_force_x , obstacle_force_y ,goal_ang, goal_dis,0)

    return v_x, v_y, v_yaw

  

  def MasterMoveCoverter(self):
    robot = self.GetRobotInfo()
    m_v, m_yaw, m_g_v_angle = self.GetMasterGlobalVector()

    angle = m_g_v_angle - robot['location']['yaw']
    x = m_v * math.cos(math.radians(angle))
    y = m_v * math.sin(math.radians(angle))
    yaw = m_yaw
    # method = self.robot.formation_info['method']
    # if(method = 'center'):
    #   x,y,yaw = self.BC.CenterFormationPoint()
    #   v_x, v_y, yaw, arrived = self.BC.Go2Point(x, y, yaw)
    # self.MotionCtrl(0, 0, 0)
    # Go2Point_cmd_vel + master_cmd_vel_to_global
    # PubCmdVel(self, x, y, yaw)

    return x, y, yaw

  def CenterFormationPoint(self):
    master = self.GetState(self.formation_info['master'])
    robot = self.GetRobotInfo()
    p_x = master['position']['x']+self.formation_info['distance']*math.cos(math.radians(master['position']['yaw']+self.formation_info['angle']))
    p_y = master['position']['y']+self.formation_info['distance']*math.sin(math.radians(master['position']['yaw']+self.formation_info['angle']))
    p_yaw = math.degrees(math.atan2(master['position']['y']-robot['location']['y'],master['position']['x']-robot['location']['x']))
    v_x, v_y, v_yaw, arrived = self.Go2Point(p_x, p_y, p_yaw)
    return v_x, v_y, v_yaw
    


  
