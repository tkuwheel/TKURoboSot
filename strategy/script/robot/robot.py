#-*- coding: UTF-8 -*-  
#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import time
import numpy as np
import time
import message_filters
import actionlib
import strategy.msg
from simple_pid import PID
from imu_3d.msg import inertia
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from vision.msg import Object
from vision.msg import Two_point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from strategy.msg import RobotState
from std_srvs.srv import *

import copy
## Rotate 90 for 6th robot
## DO NOT CHANGE THIS VALUE
ROTATE_V_ANG = 90

## Strategy Inputs
VISION_TOPIC = "vision/object"
OBSTACLE_TOPIC = "vision/obstacle"
POSITION_TOPIC = "akf_pose"
IMU            = "imu_3d"
## Strategy Outputs
STRATEGY_STATE_TOPIC = "strategy/state"
CMDVEL_TOPIC = "motion/cmd_vel"
SHOOT_TOPIC  = "motion/shoot"

class Robot(object):
  
  __twopoint_info = {'Blue':{'right' : 0,'left' : 0},
                     'Yellow':{'right' : 0,'left' : 0}}

  ball_last_time = time.time()
  sync_last_time = time.time()
  passing_last_time = time.time()

  __robot_info  = {'location' : {'x' : 0, 'y' : 0, 'yaw' : 0},
                   'imu_3d' : {'yaw' : 0}}
  __object_info = {'update_time' : 0,
                   'ball':{'dis' : 0, 'ang' : 0, 'global_x' : 0, 'global_y' : 0, \
                           'speed_x': 0, 'speed_y': 0, 'speed_pwm_x': 0, 'speed_pwm_y': 0},
                   'Blue':{'dis' : 0, 'ang' : 0},
                   'Yellow':{'dis' : 0, 'ang' : 0},
                   'velocity' : 0 }
  __obstacle_info = {'angle' : {'min' : 0, 'max' : 0, 'increment' : math.pi / 180 * 3},
		                 'scan' : 0,
		                 'range' : {'min' : 0, 'max' : 0},
                     'ranges' : [0],
		                 'intensities' : [0],
                     'detect_obstacles':[0]}
  __opp_info = {'dis':0,
                'ang':999,
                'global_x':0,
                'global_x':0}
  __ball_is_handled = False
  #======================
  __opp_is_handled = False
  __opp_is_blocked = False
  #__gobal_ball_info = {'location':{'x' : 0, 'y' : 0}}
  #======================
  robot1 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'ball_ang':0, 'obstacles':[], 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  robot2 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'ball_ang':0, 'obstacles':[], 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  robot3 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'ball_ang':0, 'obstacles':[], 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  near_robot = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'ball_ang':0, 'obstacles':[], 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  
  near_robot_ns = ""
  r1_role = ""
  r2_role = ""
  r3_role = ""

  ## Configs
  __minimum_w = 0
  __maximum_w = 0
  __minimum_v = 0
  __maximum_v = 0
  __handle_dis = 0
  __handle_ang = 0
  Kp_v = 1.62
  Ki_v = 0.0
  Kd_v = 0.1
  #Cp_v = -20
  Cp_v = 15
  Kp_w = 0.5
  Ki_w = 0.0
  Kd_w = 0.1
  Cp_w = 0

  pid_v = PID(Kp_v, Ki_v, Kd_v, setpoint=Cp_v)
  pid_v.output_limits = (-1*__maximum_v, __maximum_v)
  pid_v.auto_mode = True
  pid_w = PID(Kp_w, Ki_w, Kd_w, setpoint=Cp_w)
  pid_w.output_limits = (-1*__maximum_w, __maximum_w)
  pid_w.auto_mode = True

  def TuningVelocityContorller(self, p, i, d, cp = Cp_v):
    self.pid_v.setpoint = cp
    self.pid_v.tunings = (p, i, d)

  def TuningAngularVelocityContorller(self, p, i, d, cp = Cp_w):
    self.pid_w.setpoint = cp
    self.pid_w.tunings = (p, i, d)

  def ChangeVelocityRange(self, m, M):
    self.__minimum_v = m
    self.pid_v.output_limits = (-1*M, M)

  def ChangeAngularVelocityRange(self, m, M):
    self.__minimum_w = m
    self.pid_w.output_limits = (-1*M, M)

  def ChangeBallhandleCondition(self, dis, ang):
    self.__handle_dis = dis
    self.__handle_ang = ang

  def ShowRobotInfo(self):
    print("Robot informations: {}".format(self.__robot_info))
    print("Objects informations: {}".format(self.__object_info))
    print("Obstacles informations: {}".format(self.__obstacle_info))

  def __init__(self, sim = False):

    rospy.Subscriber(VISION_TOPIC, Object, self._GetVision)
    rospy.Subscriber(OBSTACLE_TOPIC,Int32MultiArray,self._GetObstaclesInfo)
    rospy.Subscriber(POSITION_TOPIC, PoseWithCovarianceStamped, self._GetPosition)
    rospy.Subscriber('vision/BlackRealDis', Int32MultiArray, self._GetBlackItemInfo)
    self.MotionCtrl = self.RobotCtrlS
    self.RobotShoot = self.RealShoot
    self.cmdvel_pub = self._Publisher(CMDVEL_TOPIC, Twist)
    self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC, RobotState)
    self.shoot_pub  = self._Publisher(SHOOT_TOPIC, Int32)
    robot2_sub = message_filters.Subscriber('/robot2/strategy/state', RobotState)
    robot3_sub = message_filters.Subscriber('/robot3/strategy/state', RobotState)
    ts = message_filters.ApproximateTimeSynchronizer([robot2_sub, robot3_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(self.MulticastReceiver)
    s = rospy.Service('passing_action', Trigger, self._PassingServer)

    if not sim :
      rospy.Subscriber('interface/Two_point', Two_point, self._GetTwopoint)
      rospy.Subscriber(IMU, inertia, self._GetImu)
      self.RobotBallHandle = self.RealBallHandle
    else:
      self.sim_hold_pub = rospy.Publisher('motion/hold', Bool, queue_size=1)
      self.RobotBallHandle = self.SimBallHandle
      rospy.Subscriber("BallIsHandle", Bool, self._CheckBallHandle)
      self.TuningVelocityContorller(1, 0, 0)
      self.TuningAngularVelocityContorller(0.1, 0, 0)

  def _PassingServer(self, req):
    self.MyRole = "Catcher"
    resp = TriggerResponse()
    resp.success = True
    resp.message = "Got it"
    return resp

  def PassingTo(self, catcher_ns):
    self.MyRole = "Passer"
    if catcher_ns is "nearest":
      server = self.near_robot_ns + 'passing_action'
    else:
      server = catcher_ns + 'passing_action'

    rospy.wait_for_service(server, timeout=3.0)
    try:
      client = rospy.ServiceProxy(server, Trigger)
      req = TriggerRequest()
      resp1  = client(req)
      if resp1.success:
        print("Passing Successed")
      else:
        print("Passing Failed")

      ## Pass
      self.RobotShoot(88, 0)

    except rospy.ServiceException, e:
      print("Service call failed: {}".format(e))

  def MulticastReceiver(self, r2_data, r3_data):
    Robot.sync_last_time = time.time()
    self.robot2['state']           = r2_data.state
    self.robot2['ball_is_handled'] = r2_data.ball_is_handled
    self.robot2['ball_dis']        = r2_data.ball_dis
    self.robot2['ball_ang']        = r2_data.ball_ang
    self.robot2['obstacles']       = r2_data.obstacles
    self.robot2['position']['x']   = r2_data.position.linear.x
    self.robot2['position']['y']   = r2_data.position.linear.y
    self.robot2['position']['yaw'] = r2_data.position.angular.z
    self.robot3['state']           = r3_data.state
    self.robot3['ball_is_handled'] = r3_data.ball_is_handled
    self.robot3['ball_dis']        = r3_data.ball_dis
    self.robot3['ball_ang']        = r3_data.ball_ang
    self.robot3['obstacles']       = r3_data.obstacles
    self.robot3['position']['x']   = r3_data.position.linear.x
    self.robot3['position']['y']   = r3_data.position.linear.y
    self.robot3['position']['yaw'] = r3_data.position.angular.z
    if "robot1" in rospy.get_namespace():
      dd12 = np.linalg.norm(np.array([self.__robot_info['locatoin']['x'] - self.robot2['position']['x'],
                                      self.__robot_info['location']['y'] - self.robot2['position']['y']]))
      dd13 = np.linalg.norm(np.array([self.__robot_info['locatoin']['x'] - self.robot3['position']['x'],
                                      self.__robot_info['location']['y'] - self.robot3['position']['y']]))
      self.near_robot_ns = "/robot2" if dd12 < dd13 else "/robot3"
      self.near_robot = self.robot2 if dd12 < dd13 else self.robot3
    elif "robot2" in rospy.get_namespace():
      self.near_robot_ns = "/robot3"
      # self.near_robot = self.robot3
      self.near_robot['state']           = r3_data.state
      self.near_robot['ball_is_handled'] = r3_data.ball_is_handled
      self.near_robot['ball_dis']        = r3_data.ball_dis
      self.near_robot['ball_ang']        = r3_data.ball_ang
      self.near_robot['obstacles']       = r3_data.obstacles
      self.near_robot['position']['x']   = r3_data.position.linear.x
      self.near_robot['position']['y']   = r3_data.position.linear.y
      self.near_robot['position']['yaw'] = r3_data.position.angular.z
    elif "robot3" in rospy.get_namespace():
      self.near_robot_ns = "/robot2"
      # self.near_robot = self.robot2
      self.near_robot['state']           = r2_data.state
      self.near_robot['ball_is_handled'] = r2_data.ball_is_handled
      self.near_robot['ball_dis']        = r2_data.ball_dis
      self.near_robot['ball_ang']        = r2_data.ball_ang
      self.near_robot['obstacles']       = r2_data.obstacles
      self.near_robot['position']['x']   = r2_data.position.linear.x
      self.near_robot['position']['y']   = r2_data.position.linear.y
      self.near_robot['position']['yaw'] = r2_data.position.angular.z
    #--------------
    
   
  def Supervisor(self):
    duration = time.time() - Robot.sync_last_time
    if duration > 2:
      #print("Lossing Connection with teammates...{}".format(duration), end='\r')
      self.SetMyRole(rospy.get_param('core/role'))
    else:
      r_x = self.near_robot['position']['x']
      r_y = self.near_robot['position']['y']
      # print("r_x, r_y", r_x, r_y)
      #=============
      if(self.__object_info['ball']['ang']==999 and self.near_robot['ball_ang']<999 and self.near_robot['ball_dis']<999):
        # r_x = self.near_robot['position']['x']
        # r_y = self.near_robot['position']['y']
        # print("r_x, r_y", r_x, r_y)
        rbx = self.near_robot['ball_dis'] * math.cos(math.radians(self.near_robot['ball_ang']))
        rby = self.near_robot['ball_dis'] * math.sin(math.radians(self.near_robot['ball_ang']))
        rrbx, rrby = self.Rotate(rbx, rby, self.near_robot['position']['yaw'])
        gbx = rrbx + self.near_robot['position']['x']
        gby = rrby + self.near_robot['position']['y']
        b_dis = math.hypot(gby - self.__robot_info['location']['y'], gbx - self.__robot_info['location']['x'])
        ang_tmp = math.atan2(gby - self.__robot_info['location']['y'], gbx - self.__robot_info['location']['x'])
        b_ang = math.degrees(ang_tmp)-self.__robot_info['location']['yaw']
        # print("b_dis", int(b_dis) , self.__object_info['ball']['dis'])
        # print("b_ang",int(b_ang) , self.__object_info['ball']['ang'])
        if(b_ang>180):
          b_ang=b_ang-360
        elif(b_ang<(-180)):
          b_ang=b_ang+360
        self.__object_info['ball']['dis'] = b_dis
        self.__object_info['ball']['ang'] = b_ang
      #=============
      if self.MyRole() is "Catcher" or self.MyRole is "Passer":
        if self.robot2['ball_is_handled']:
          self.r2_role = "Attacker"
          self.r3_role = "Supporter"
        elif self.robot3['ball_is_handled']:
          self.r2_role = "Supporter"
          self.r3_role = "Attacker"
        elif time.time() - Robot.passing_last_time > 5:
          ## Passing Timeout
          self.r2_role = "Attacker" if self.robot2['ball_dis'] < self.robot3['ball_dis'] else "Supporter"
          self.r3_role = "Supporter" if self.r2_role is "Attacker" else "Attacker"
      else:
        if self.robot2['ball_is_handled']:
          self.r2_role = "Attacker"
          self.r3_role = "Supporter"
        elif self.robot3['ball_is_handled']:
          self.r2_role = "Supporter"
          self.r3_role = "Attacker"
        else:
          self.r2_role = "Attacker" if self.robot2['ball_dis'] < self.robot3['ball_dis'] else "Supporter"
          self.r3_role = "Supporter" if self.r2_role is "Attacker" else "Attacker"
        if(self.robot2['state']=='idle'):
          self.r3_role = "Attacker"
        if(self.robot3['state']=='idle'):
          self.r2_role = "Attacker"

  def GetState(self, robot_ns):
    if "robot1" in robot_ns.lower():
      return self.robot1
    elif "robot2" in robot_ns.lower():
      return self.robot2
    elif "robot3" in robot_ns.lower():
      return self.robot3
    elif "near_robot" in robot_ns.lower():
      # print(self.near_robot)
      return self.near_robot
    else:
      print("Wrong Namespace")

  def MyState(self):
    if "robot1" in rospy.get_namespace():
      return self.robot1
    elif "robot2" in rospy.get_namespace():
      return self.robot2
    elif "robot3" in rospy.get_namespace():
      return self.robot3
    else:
      print("Wrong Namespace")

  def MyRole(self):
    if "robot1" in rospy.get_namespace():
      return self.r1_role
    elif "robot2" in rospy.get_namespace():
      return self.r2_role
    elif "robot3" in rospy.get_namespace():
      return self.r3_role
    else:
      return self.r1_role
      print("Wrong Namespace")
      return "Wrong Namespace"

  def SetMyRole(self, role):
    if "robot1" in rospy.get_namespace():
      self.r1_role = role
    elif "robot2" in rospy.get_namespace():
      self.r2_role = role
    elif "robot3" in rospy.get_namespace():
      self.r3_role = role
    else:
      self.r1_role = role
      print("Wrong Namespace")

  def _Publisher(self, topic, mtype):
    return rospy.Publisher(topic, mtype, queue_size=1)

  def _GetVision(self, vision):
    self.__object_info['update_time'] = time.time()
    rbx = vision.ball_dis * math.cos(math.radians(vision.ball_ang))
    rby = vision.ball_dis * math.sin(math.radians(vision.ball_ang))
    rrbx, rrby = self.Rotate(rbx, rby, self.__robot_info['location']['yaw'])
    gbx = rrbx + self.__robot_info['location']['x']
    gby = rrby + self.__robot_info['location']['y']
    if time.time() - Robot.ball_last_time >= 0.5:
      spx = (gbx - self.__object_info['ball']['global_x']) * 0.02 # m / 0.5s
      spy = (gby - self.__object_info['ball']['global_y']) * 0.02
      spwmx, spwmy = self.ConvertSpeedToPWM(spx, spy)
      self.__object_info['ball']['speed_x']  = spx
      self.__object_info['ball']['speed_y']  = spy
      self.__object_info['ball']['speed_pwm_x']  = spwmx
      self.__object_info['ball']['speed_pwm_y']  = spwmy
      # self.__object_info['ball']['global_x'] = gbx
      # self.__object_info['ball']['global_y'] = gby
      # print(spwmx, spwmy)
      Robot.ball_last_time = time.time()
    if(vision.ball_ang!=999):
      self.__object_info['ball']['global_x'] = gbx
      self.__object_info['ball']['global_y'] = gby
    self.__object_info['ball']['dis']    = vision.ball_dis
    self.__object_info['ball']['ang']    = vision.ball_ang
    self.__object_info['Blue']['dis']    = vision.blue_fix_dis
    self.__object_info['Blue']['ang']    = vision.blue_fix_ang
    self.__object_info['Yellow']['dis']  = vision.yellow_fix_dis
    self.__object_info['Yellow']['ang']  = vision.yellow_fix_ang
    
    if self.__object_info['ball']['dis'] <= self.__handle_dis and abs(self.__object_info['ball']['ang']) <= self.__handle_ang:
      self.__ball_is_handled = True
    else:
      self.__ball_is_handled = False

  def Angle_Adjustment(self, ang):
    if(ang > 180):
        ang = 360-ang
    elif(ang < (-180)):
        ang = ang + 360
    return ang

  def obstacle_fileter(self, obs, robot, near_robot=None):
    obs_filter = []
    info_time = Robot.sync_last_time
    now = time.time()
    dt = abs(now-info_time)
    # print(Robot.sync_last_time)
    # print("now", time.time())
    # print("dt", dt)
    r_x = 999
    r_y = 999
    if(near_robot is not None):
        r_x = near_robot['position']['x']
        r_y = near_robot['position']['y']
        r_yaw = near_robot['position']['yaw']
    # print(r_x, r_y)
    for i in range (0,len(obs), 4):
        if(len(obs)==0):
          break
        distance = obs[i+0]
        angle    = obs[i+1]+robot["location"]["yaw"]
        o_x      = robot["location"]["x"] + distance * math.cos(math.radians(angle))
        o_y      = robot["location"]["y"] + distance * math.sin(math.radians(angle))
        dis      = math.sqrt(math.pow((r_x-o_x),2)+math.pow((r_y-o_y),2))
        #未接到隊友資訊
        ang_tmp = math.atan2(r_y - robot['location']['y'], r_x - robot['location']['x'])
        r_angle = self.Angle_Adjustment(math.degrees(ang_tmp)-robot['location']['yaw'])
        abs_yaw = abs(r_angle-obs[i+1])
        if(dt>2 or near_robot==None):
            dis = 999
            abs_yaw=999
        # print("dis", dis)
        # print(o_x, o_y)
        # print ("abs_yaw", abs_yaw, r_angle, obs[i+1])
        if(abs(o_y)<200 and abs_yaw>30):
            obs_filter.append(obs[i+0])
            obs_filter.append(obs[i+1])
            obs_filter.append(obs[i+2])
            obs_filter.append(obs[i+3])
    return obs_filter
  def _GetObstaclesInfo(self,obs_info):
    self.__obstacle_info['detect_obstacles'] = obs_info.data
    obs = self.__obstacle_info['detect_obstacles']
    #============obs_filter===============
    robot_info = self.GetRobotInfo()
    obstacles_info = self.GetObstacleInfo()
    obs = obstacles_info["detect_obstacles"]
    obs_filter = self.obstacle_fileter(obs, robot_info, self.near_robot)
    #=====================================
    opp_handled_ang = 10
    opp_handled_dis = 40

    for i in range (0,len(obs_filter), 4):
      dis = obs_filter[i+0]
      ang = obs_filter[i+1]
      left_ang = obs_filter[i+2]
      right_ang = obs_filter[i+3]
      abs_mid_ang = abs(self.__object_info['ball']['ang']-ang)
      abs_mid_ang = min(abs_mid_ang, abs(360-abs_mid_ang))
      abs_left_ang = abs(self.__object_info['ball']['ang']-left_ang)
      abs_left_ang = min(abs_left_ang, abs(360-abs_left_ang))
      abs_right_ang = abs(self.__object_info['ball']['ang']-right_ang)
      abs_right_ang = min(abs_right_ang, abs(360-abs_right_ang))
      
      min_ang = min(abs_mid_ang, abs_left_ang, abs_right_ang)
      if(min_ang < opp_handled_ang and self.__object_info['ball']['dis']<dis):
        self.__opp_is_blocked = True
      if(min_ang < opp_handled_ang and abs(self.__object_info['ball']['dis']-dis) < opp_handled_dis):
        #print(min_ang, self.__object_info['ball']['dis']-dis)
        self.__opp_is_handled = True
        if(self.__object_info['ball']['ang']<999):
            self.__opp_info['ang']=ang
            self.__opp_info['dis']=dis
        break
      if(i == (len(obs_filter)-4)):
        self.__opp_is_handled = False
        self.__opp_is_blocked = False
    # if(self.__object_info['ball']['ang']==999):
    #   print("can't see ball")
    dt = time.time() - Robot.sync_last_time
    if(self.__ball_is_handled or(dt<2 and self.near_robot['ball_is_handled']==True)):
      self.__opp_is_handled = False
      self.__opp_is_blocked = False
    #print("self.__opp_is_handled", self.__opp_is_handled)
    #print("self.__opp_is_blocked", self.__opp_is_blocked)

    #如果找不到球 搜尋球最後大地座標周圍障礙物 若找到障礙物將其當作持球員 並以持球員大地座標座代替球大地座標
    if(self.__object_info['ball']['ang']<999 and self.__opp_is_handled == True):
      ob_max_dis = opp_handled_dis
      min_ob_dis = 999

      ball_dis = self.__object_info['ball']['dis']
      ball_ang = self.__object_info['ball']['ang']

      gbx = self.__object_info['ball']['global_x']
      gby = self.__object_info['ball']['global_y']

      self.__opp_info['ang']=999
      self.__opp_info['dis']=0
      for i in range (0,len(obs_filter), 4):
        dis = obs_filter[i+0]
        ang = obs_filter[i+1]
        #sas A2=B2+C2-BCcos(A)
        rox = dis * math.cos(math.radians(ang))
        roy = dis * math.sin(math.radians(ang))
        rrox, rroy = self.Rotate(rox, roy, self.__robot_info['location']['yaw'])
        gox = rrox + self.__robot_info['location']['x']
        goy = rroy + self.__robot_info['location']['y']
        ob_dis = math.sqrt(math.pow(gox-gbx,2)+math.pow(goy-gby,2))
        if(ob_dis<ob_max_dis and ob_dis<min_ob_dis):
          self.__opp_info['ang']=ang
          self.__opp_info['dis']=dis
          self.__object_info['ball']['global_x']=gox
          self.__object_info['ball']['global_y']=goy
    #=====================

  def _GetTwopoint(self,vision):
    self.__twopoint_info['Blue']['right']   = vision.blue_right
    self.__twopoint_info['Blue']['left']    = vision.blue_left
    self.__twopoint_info['Yellow']['right'] = vision.yellow_right
    self.__twopoint_info['Yellow']['left']  = vision.yellow_left

  def _GetBlackItemInfo(self, vision):
    self.__obstacle_info['ranges'] =vision.data

  def _GetImu(self, imu_3d):
    front_ang = math.degrees(imu_3d.yaw) + 90 
    self.__robot_info['imu_3d']['yaw'] = imu_3d.yaw  #caculate front angle by imu

  def _GetPosition(self,loc):
    self.__robot_info['location']['x'] = loc.pose.pose.position.x*100
    self.__robot_info['location']['y'] = loc.pose.pose.position.y*100
    qx = loc.pose.pose.orientation.x
    qy = loc.pose.pose.orientation.y
    qz = loc.pose.pose.orientation.z
    qw = loc.pose.pose.orientation.w
    self.__robot_info['location']['yaw'] = math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)\
                                           / math.pi * 180  #caculate front angle by self_localization
                                                                                                                                         
  def RobotStatePub(self, state):
    m = RobotState()
    m.state = state
    m.ball_is_handled = self.__ball_is_handled
    m.ball_dis = self.__object_info['ball']['dis']
    m.ball_ang = self.__object_info['ball']['ang']
    m.obstacles = self.__obstacle_info['detect_obstacles']
    m.position.linear.x  = self.__robot_info['location']['x']
    m.position.linear.y  = self.__robot_info['location']['y']
    m.position.angular.z = self.__robot_info['location']['yaw']
    if(abs(self.__object_info['update_time']-time.time())>1):
      m.ball_dis = 999
      m.ball_ang = 999
    self.state_pub.publish(m)

  def ConvertSpeedToPWM(self, x, y):
    reducer = 24
    max_rpm = 7580
    wheel_radius  = 0.11
    circumference = 2 * math.pi * wheel_radius
    _x = (x / circumference * reducer * 60)/max_rpm * 100
    _y = (y / circumference * reducer * 60)/max_rpm * 100
    return _x, _y

  def Rotate(self, x, y, theta):
    _x = x*math.cos(math.radians(theta)) - y*math.sin(math.radians(theta))
    _y = x*math.sin(math.radians(theta)) + y*math.cos(math.radians(theta))
    return _x, _y

  def ConvertSpeedToPWM(self, x, y):
    reducer = 24
    max_rpm = 7580
    wheel_radius  = 0.11
    circumference = 2 * math.pi * wheel_radius
    _x = (x / circumference * reducer * 60)/max_rpm * 100
    _y = (y / circumference * reducer * 60)/max_rpm * 100
    return _x, _y

  def RobotCtrlS(self, x, y, yaw, pass_through=False , max_v = 100):
    if pass_through:
      msg = Twist()
      output_x, output_y = self.Rotate(x, y, ROTATE_V_ANG)
      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = yaw
      print("Orbit",output_x, output_y, yaw)
      self.cmdvel_pub.publish(msg)
    else:
      current_vector = math.hypot(x, y)
      output_v = self.pid_v(current_vector * -1)
      output_w = self.pid_w(yaw) * -1
      output_w = output_w if abs(output_w) > self.__minimum_w else self.__minimum_w* np.sign(output_w)

      magnitude = math.sqrt(x**2 + y**2)
      if magnitude == 0:
        unit_vector = (0, 0)
      else:
        unit_vector = (x / magnitude, y / magnitude)

      msg = Twist()
      output_x, output_y = self.Rotate(unit_vector[0]*output_v, unit_vector[1]*output_v, ROTATE_V_ANG)
      if(x==0 and y==0 and yaw==0):
        output_x=0
        output_y=0
        output_w=0
      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = output_w
      if(abs(msg.linear.x)>max_v ):
        if(msg.linear.x<0):
          msg.linear.x = -max_v
        else:
          msg.linear.x = max_v
      if(abs(msg.linear.y)>max_v ):
        if(msg.linear.y<0):
          msg.linear.y = -max_v
        else:
          msg.linear.y = max_v
      self.cmdvel_pub.publish(msg)

  def GetObjectInfo(self):
    return self.__object_info

  def GetRobotInfo(self):
    return self.__robot_info
 
  def GetTwopoint(self):
    return self.__twopoint_info

  def GetObstacleInfo(self):
    return self.__obstacle_info

  def RealShoot(self, power, pos) :
    msg = Int32()
    msg.data = power
    self.shoot_pub.publish(msg)

  def SimBallHandle(self):
    self.sim_hold_pub.publish(True)
    return self.__ball_is_handled

  def _CheckBallHandle(self, data):
    self.__ball_is_handled = data.data

  def RealBallHandle(self):
    return self.__ball_is_handled

  def GetOppHandle(self):
    return self.__opp_is_handled

  def GetOppBlock(self):
    return self.__opp_is_blocked
  
  def GetOppInfo(self):
    return self.__opp_info
