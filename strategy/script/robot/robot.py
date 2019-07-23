#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
from simple_pid import PID
from imu_3d.msg import inertia
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from vision.msg import Object
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

## Rotate 90 for 6th robot
## DO NOT CHANGE THIS VALUE
ROTATE_V_ANG = 90

## Strategy Inputs
VISION_TOPIC = "vision/object"
POSITION_TOPIC = "akf_pose"
IMU            = "imu_3d"

## Strategy Outputs
STRATEGY_STATE_TOPIC = "strategy/state"
CMDVEL_TOPIC = "motion/cmd_vel"
SHOOT_TOPIC  = "motion/shoot"

class Robot(object):

  last_time = 0

  __robot_info  = {'location' : {'x' : 0, 'y' : 0, 'yaw' : 0},
                   'imu_3d' : {'yaw' : 0}}
  __object_info = {'ball':{'dis' : 0, 'ang' : 0, 'global_x' : 0, 'global_y' : 0, \
                           'speed_x': 0, 'speed_y': 0, 'speed_pwm_x': 0, 'speed_pwm_y': 0},
                   'Blue':{'dis' : 0, 'ang' : 0},
                   'Yellow':{'dis' : 0, 'ang' : 0},
                   'velocity' : 0 }
  __obstacle_info = {'angle' : {'min' : 0, 'max' : 0, 'increment' : math.pi / 180 * 3},
		                 'scan' : 0,
		                 'range' : {'min' : 0, 'max' : 0},
                     'ranges' : [0],
		                 'intensities' : [0]}

  __ball_is_handled = False
  ## Configs
  __minimum_w = 0
  __maximum_w = 0
  __minimum_v = 0
  __maximum_v = 0
  __handle_dis = 0
  __handle_ang = 0
  Kp_v = 1.5
  Ki_v = 0.0
  Kd_v = 0.1
  Cp_v = 0
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

  def __init__(self, robot_num, sim = False):
    self.robot_number = robot_num

    rospy.Subscriber(VISION_TOPIC, Object, self._GetVision)
    rospy.Subscriber(POSITION_TOPIC,PoseWithCovarianceStamped,self._GetPosition)
    rospy.Subscriber('BlackRealDis',Int32MultiArray,self._GetBlackItemInfo)
    self.MotionCtrl = self.RobotCtrlS
    self.RobotShoot = self.RealShoot
    self.cmdvel_pub = self._Publisher(CMDVEL_TOPIC, Twist)
    self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC.format(self.robot_number), String)
    self.shoot_pub  = self._Publisher(SHOOT_TOPIC, Int32)

    if not sim :
      rospy.Subscriber(IMU,inertia,self._GetImu)
      self.RobotBallHandle = self.RealBallHandle
    else:
      self.sim_hold_pub = rospy.Publisher('motion/hold', Bool, queue_size=1)
      self.RobotBallHandle = self.SimBallHandle
      rospy.Subscriber("BallIsHandle", Bool, self._CheckBallHandle)
      self.TuningVelocityContorller(1, 0, 0)
      self.TuningAngularVelocityContorller(0.1, 0, 0)

  def _Publisher(self, topic, mtype):
    return rospy.Publisher(topic, mtype, queue_size=1)

  def _GetVision(self, vision):
    rbx = vision.ball_dis * math.cos(math.radians(vision.ball_ang))
    rby = vision.ball_dis * math.sin(math.radians(vision.ball_ang))
    rrbx, rrby = self.Rotate(rbx, rby, self.__robot_info['location']['yaw'])
    gbx = rrbx + self.__robot_info['location']['x']
    gby = rrby + self.__robot_info['location']['y']
    if time.time() - Robot.last_time >= 0.5:
      spx = (gbx - self.__object_info['ball']['global_x']) * 0.02 # m / 0.5s
      spy = (gby - self.__object_info['ball']['global_y']) * 0.02
      spwmx, spwmy = self.ConvertSpeedToPWM(spx, spy)
      self.__object_info['ball']['speed_x']  = spx
      self.__object_info['ball']['speed_y']  = spy
      self.__object_info['ball']['speed_pwm_x']  = spwmx
      self.__object_info['ball']['speed_pwm_y']  = spwmy
      self.__object_info['ball']['global_x'] = gbx
      self.__object_info['ball']['global_y'] = gby
      # print(spwmx, spwmy)
      Robot.last_time = time.time()

    self.__object_info['ball']['dis']    = vision.ball_dis
    self.__object_info['ball']['ang']    = vision.ball_ang
    self.__object_info['Blue']['dis']    = vision.blue_fix_dis
    self.__object_info['Blue']['ang']    = vision.blue_fix_ang
    self.__object_info['Yellow']['dis']  = vision.yellow_fix_dis
    self.__object_info['Yellow']['ang']  = vision.yellow_fix_ang

  def _GetBlackItemInfo(self, vision):
    self.__obstacle_info['ranges'] =vision.data

  def _GetImu(self, imu_3d):
    self.__robot_info['imu_3d']['yaw'] = imu_3d.yaw

  def _GetPosition(self,loc):
    self.__robot_info['location']['x'] = loc.pose.pose.position.x*100
    self.__robot_info['location']['y'] = loc.pose.pose.position.y*100
    qx = loc.pose.pose.orientation.x
    qy = loc.pose.pose.orientation.y
    qz = loc.pose.pose.orientation.z
    qw = loc.pose.pose.orientation.w
    self.__robot_info['location']['yaw'] = math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) / math.pi * 180

  def RobotStatePub(self, state):
    s = String()
    s.data = state
    self.state_pub.publish(s)

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

  def RobotCtrlS(self, x, y, yaw, pass_through=False):
    if pass_through:
      msg = Twist()
      output_x, output_y = self.Rotate(x, y, ROTATE_V_ANG)
      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = yaw
      #print(output_x, output_y, yaw)
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

      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = output_w
      self.cmdvel_pub.publish(msg)

  def GetObjectInfo(self):
    return self.__object_info

  def GetRobotInfo(self):
    return self.__robot_info
  
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
    if self.__object_info['ball']['dis'] <= self.__handle_dis and  self.__object_info['ball']['ang'] <= self.__handle_ang:
     
      return True
    else:
      return False
