#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import Shoot
from nubot_common.srv import BallHandle
from transfer.msg import PPoint
from simple_pid import PID
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from vision.msg import Object
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# General Configs
MINIMUM_W = 0.2
MAXIMUM_W = 100

# Gazebo Simulator
SIM_VISION_TOPIC = "nubot{}/omnivision/OmniVisionInfo"
SIM_CMDVEL_TOPIC = "nubot{}/nubotcontrol/velcmd"
SIM_SHOOT_SRV  = "nubot{}/Shoot"
SIM_HANDLE_SRV = "nubot{}/BallHandle"

# Real Robot
VISION_TOPIC = "vision/object"
CMDVEL_TOPIC = "motion/cmd_vel"

# Strategy Outputs
STRATEGY_STATE_TOPIC = "robot{}/strategy/state"

class Robot(object):

  __robot_info = {'location' : {'x' : 0, 'y' : 0, 'yaw' : 0}}
  __object_info = {'ball':{'dis' : 0, 'ang' : 0},
                   'Cyan':{'dis' : 0, 'ang' : 0},
                   'Magenta':{'dis' : 0, 'ang' : 0},
                   'velocity' : 0 }
  pid_v = PID(-1.5, 0, 0, setpoint=10)
  pid_v.output_limits = (0, 100)
  pid_v.auto_mode = True
  pid_w = PID(0.4, 0.0, 0.0, setpoint=0) # Motor: 241370
  pid_w.output_limits = (-1*MAXIMUM_W, MAXIMUM_W)
  pid_w.auto_mode = True

  def ShowRobotInfo(self):
    print("Robot informations: {}".format(self.__robot_info))
    print("Objects informations: {}".format(self.__object_info))

  def __init__(self, robot_num, sim = False):
    self.robot_number = robot_num

    if not sim :
      rospy.Subscriber(VISION_TOPIC, Object, self._GetVision)
      self.cmdvel_pub = self._Publisher(CMDVEL_TOPIC, Twist)
      self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC.format(self.robot_number), String)
      self.MotionCtrl = self.RobotCtrlS
      self.RobotBallHandle = self.RealBallHandle
      self.RobotShoot = self.SimShoot
    else:
      self._SimSubscriber(SIM_VISION_TOPIC.format(self.robot_number))
      self.cmdvel_pub = self._Publisher(SIM_CMDVEL_TOPIC.format(self.robot_number), VelCmd)
      self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC.format(self.robot_number), String)
      self.MotionCtrl = self.RobotCtrlS
      self.RobotBallHandle = self.SimBallHandle
      self.RobotShoot = self.RealShoot

  def _SimSubscriber(self, topic):
    rospy.Subscriber(topic.format(self.robot_number), \
                      OminiVisionInfo, \
                      self._GetSimVision)
    rospy.Subscriber((topic + "/GoalInfo").format(self.robot_number), \
                      PPoint, \
                      self._GetSimGoalInfo)

  def _Publisher(self, topic, mtype):
    return rospy.Publisher(topic.format(self.robot_number), mtype, queue_size=1)

  def _GetSimVision(self, vision):
    self.__object_info['ball']['dis'] = vision.ballinfo.real_pos.radius
    self.__object_info['ball']['ang'] = math.degrees(vision.ballinfo.real_pos.angle)
    self.__robot_info['location']['x']   = vision.robotinfo[self.robot_number - 1].pos.x
    self.__robot_info['location']['y']   = vision.robotinfo[self.robot_number - 1].pos.y
    self.__robot_info['location']['yaw'] = math.degrees(vision.robotinfo[self.robot_number - 1].heading.theta)

  def _GetSimGoalInfo(self, goal_info):
    self.__object_info['Cyan']['dis'] = goal_info.left_radius
    self.__object_info['Cyan']['ang'] = goal_info.left_angle
    self.__object_info['Magenta']['dis'] = goal_info.right_radius
    self.__object_info['Magenta']['ang'] = goal_info.right_angle

  def _GetVision(self, vision):
    self.__object_info['ball']['dis']    = vision.ball_dis
    self.__object_info['ball']['ang']    = vision.ball_ang
    self.__object_info['Cyan']['dis']    = vision.blue_fix_dis
    self.__object_info['Cyan']['ang']    = vision.blue_fix_ang
    self.__object_info['Magenta']['dis'] = vision.yellow_fix_dis
    self.__object_info['Magenta']['ang'] = vision.yellow_fix_ang

  def RobotStatePub(self, state):
    s = String()
    s.data = state
    self.state_pub.publish(s)

  def _Rotate(self, x, y, theta):
    _x = x*cos(math.radians(theta)) - y*sin(math.radians(theta))
    _y = x*sin(math.radians(theta)) + y*cos(math.radians(theta))
    return _x, _y

  def RobotCtrlS(self, x, y, yaw, pass_through=False):
    if pass_through:
      msg = Twist()
      msg.linear.x = -y
      msg.linear.y = x
      msg.angular.z  = yaw
      #msg = VelCmd()
      #msg.Vx = x
      #msg.Vy = y
      #msg.w  = yaw
      self.cmdvel_pub.publish(msg)
    else:
      current_vector = math.hypot(x, y)
      output_v = self.pid_v(current_vector)
      output_w = self.pid_w(yaw) * -1
      output_w = output_w if abs(output_w) > MINIMUM_W else MINIMUM_W * np.sign(output_w)

      magnitude = math.sqrt(x**2 + y**2)
      if magnitude == 0:
        unit_vector = (0, 0)
      else:
        unit_vector = (x / magnitude, y / magnitude)

      # print(yaw)
      # print("Output: ",(unit_vector[0]*output_v, unit_vector[1]*output_v, output_w))
      #msg = VelCmd()
      #msg.Vx = unit_vector[1]*output_v * -1
      #msg.Vy = unit_vector[0]*output_v
      #msg.Vx = 0
      #msg.Vy = 0
      #msg.w  = output_w
      msg = Twist()
      msg.linear.x = 0
      msg.linear.y = 0
      msg.angular.z  = output_w
      self.cmdvel_pub.publish(msg)

  def RobotCtrl(self, x, y, yaw):
    angle = yaw
    velocity = math.hypot(x, y)
    if x != 0:
      alpha = math.degrees(math.atan2(y, x))
    else:
      alpha = 0

    dis_max = 2
    dis_min = 0.3
    velocity_max = 40
    velocity_min = 20
    angular_velocity_max = 4.5
    angular_velocity_min = 2
    angle_max = 144
    angle_min = 20
    angle_out = angle
    if velocity == 0:
      pass
    elif velocity > dis_max:
      velocity = velocity_max
    elif velocity < dis_min:
      velocity = velocity_min
    else:
      velocity = (velocity_max - velocity_min) * \
                 (math.cos((((velocity - dis_min) / (dis_max-dis_min) - 1) * math.pi)) + 1 )/ 2 + velocity_min
    if angle == 0:
      pass
    elif abs(angle) > angle_max:
      angle_out = angular_velocity_max
    elif abs(angle) < angle_min:
      angle_out = angular_velocity_min
    else:
      angle_out = (angular_velocity_max - angular_velocity_min) * \
                  (math.cos((((angle - angle_min) / (angle_max-angle_min) - 1) * math.pi)) + 1 )/ 2 + angular_velocity_min
    if angle < 0:
      angle_out = -angle_out
    x = velocity * math.cos(math.radians(alpha))
    y = velocity * math.sin(math.radians(alpha))
    yaw = angle_out

    msg = Twist()
    msg.linear.x = -y
    msg.linear.y = x
    msg.angular.z = yaw

    self.cmdvel_pub.publish(msg)

  def GazeboCtrl(self, x, y, yaw):
    angle = yaw
   
    velocity = math.hypot(x, y)
    if x != 0:
      alpha = math.degrees(math.atan2(y, x))
    else:
      alpha = 0

    dis_max = 2
    dis_min = 0.3
    velocity_max = 80
    velocity_min = 20
    angular_velocity_max = 4.5
    angular_velocity_min = 2
    angle_max = 144
    angle_min = 20
    angle_out = angle
    if velocity == 0:
      pass
    elif velocity > dis_max:
      velocity = velocity_max
    elif velocity < dis_min:
      velocity = velocity_min
    else:
      velocity = (velocity_max - velocity_min) * \
                 (math.cos((((velocity - dis_min) / (dis_max-dis_min) - 1) * math.pi)) + 1 )/ 2 + velocity_min
    if angle == 0:
      pass
    elif abs(angle) > angle_max:
      angle_out = angular_velocity_max
    elif abs(angle) < angle_min:
      angle_out = angular_velocity_min
    else:
      angle_out = (angular_velocity_max - angular_velocity_min) * \
                  (math.cos((((angle - angle_min) / (angle_max-angle_min) - 1) * math.pi)) + 1 )/ 2 + angular_velocity_min
    if angle < 0:
      angle_out = -angle_out
    x = velocity * math.cos(math.radians(alpha))
    y = velocity * math.sin(math.radians(alpha))
    yaw = angle_out

    msg = VelCmd()
    msg.Vx = x
    msg.Vy = y
    msg.w = yaw
    
    self.cmdvel_pub.publish(msg)

  def GetObjectInfo(self):
    return self.__object_info

  def GetRobotInfo(self):
    return self.__robot_info

  def SimShoot(self, power, pos) :
    rospy.wait_for_service(SIM_SHOOT_SRV.format(self.robot_number))
    try:
      client = rospy.ServiceProxy(SIM_SHOOT_SRV.format(self.robot_number), Shoot)
      resp1 = client(power, pos)
      return resp1
    except rospy.ServiceException :
      print ("Service call failed")

  def RealShoot(self, power, pos) :
    pass

  def SimBallHandle(self):
    rospy.wait_for_service(SIM_HANDLE_SRV.format(self.robot_number))
    try:
      client = rospy.ServiceProxy(SIM_HANDLE_SRV.format(self.robot_number), BallHandle)
      resp1 = client(1)
      return resp1.BallIsHolding
    except rospy.ServiceException :
      print ("Service call failed")

  def RealBallHandle(self):
    if self.__object_info['ball']['dis'] < 30:
      print("Ball Handled")
      return True
    else:
      return False
