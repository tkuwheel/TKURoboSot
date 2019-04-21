import rospy
import math
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import Shoot
from nubot_common.srv import BallHandle
from transfer.msg import PPoint

SIM_VISION_TOPIC = "nubot{}/omnivision/OmniVisionInfo"
SIM_CMDVEL_TOPIC = "nubot{}/nubotcontrol/velcmd"

class Robot(object):

  __robot_info = {'location' : None}
  __object_info = {'ball':{'dis' : 0, 'ang' : 0},
                   'cyan_goal':{'dis' : 0, 'ang' : 0},
                   'magenta_goal':{'dis' : 0, 'ang' : 0}}

  def ShowRobotInfo(self):
    print("Robot informations: {}".format(self.__robot_info))
    print("Objects informations: {}".format(self.__object_info))

  def __init__(self, robot_num, sim = False):
    self.robot_number = robot_num

    if not sim :
      self._Subscriber("")
      self._Publisher("")
    else:
      self._Subscriber(SIM_VISION_TOPIC.format(self.robot_number))
      self._Publisher(SIM_CMDVEL_TOPIC.format(self.robot_number))

  def _Subscriber(self, topic):
    rospy.Subscriber(topic.format(self.robot_number), \
                      OminiVisionInfo, \
                      self._GetOmniVsison)
    rospy.Subscriber((topic + "/GoalInfo").format(self.robot_number), \
                      PPoint, \
                      self._GetGoalInfo)

  def _Publisher(self, topic):
    self.cmdvel_pub = rospy.Publisher(topic.format(self.robot_number), \
                                      VelCmd, \
                                      queue_size=100)

  def _GetOmniVsison(self, vision):
    self.__object_info['ball']['dis'] = vision.ballinfo.real_pos.radius
    self.__object_info['ball']['ang'] = math.degrees(vision.ballinfo.real_pos.angle)
    
  def _GetGoalInfo(self, goal_info):
    self.__object_info['cyan_goal']['dis'] = goal_info.left_radius
    self.__object_info['cyan_goal']['ang'] = goal_info.left_angle
    self.__object_info['magenta_goal']['dis'] = goal_info.right_radius
    self.__object_info['magenta_goal']['ang'] = goal_info.right_angle

  def RobotCtrl(self, x, y, yaw):
    angle = yaw
    velocity = math.hypot(x, y)
    if x != 0:
        alpha = math.degrees(math.atan2(y, x))
    else:
        alpha = 0

    dis_max = 2
    dis_min = 0.3
    velocity_max = 70
    velocity_min = 50
    angular_velocity_max = 2
    angular_velocity_min = 0.5
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
        velocity = (velocity_max - velocity_min) * (math.cos((((velocity - dis_min) / (dis_max-dis_min) - 1) * math.pi)) + 1 )/ 2 + velocity_min
    if angle == 0:
        pass
    elif abs(angle) > angle_max:
        angle_out = angular_velocity_max
    elif abs(angle) < angle_min:
        angle_out = angular_velocity_min
    else:
        angle_out = (angular_velocity_max - angular_velocity_min) * (math.cos((((angle - angle_min) / (angle_max-angle_min) - 1) * math.pi)) + 1 )/ 2 + angular_velocity_min
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

  def shoot(self,x,y) :
    rospy.wait_for_service('nubot1/Shoot')
    try:
      Shoot_client = rospy.ServiceProxy('nubot1/Shoot',Shoot)
      resp1 = Shoot_client(x,y)
      return resp1
    except(rospy.ServiceException):
      print("Service call failed")

  
  def ballhandle(self):
    rospy.wait_for_service('nubot1/Ballhandle')
    try:
      Ballhandle_client = rospy.ServiceProxy('nubot1/Ballhandle',BallHandle)
      resp1 = Ballhandle_client(1)
      return resp1
    except(rospy.ServiceException):
      print("Service call failed") 
  
