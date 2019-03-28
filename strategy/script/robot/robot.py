import rospy
import math
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from transfer.msg import PPoint

SIM_VISION_TOPIC = "nubot{}/omnivision/OmniVisionInfo"
SIM_CMDVEL_TOPIC = "nubot{}/nubotcontrol/velcmd"

class Robot(object):

  __robot_info = {'location' : None}
  __object_info = {'ball':{'dis' : None, 'ang' : None},
                   'cyan_goal':{'dis' : None, 'ang' : None},
                   'magenta_goal':{'dis' : None, 'ang' : None}}

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
    msg = VelCmd()
    msg.Vx = x
    msg.Vy = y
    msg.w = yaw
    self.cmdvel_pub.publish(msg)

  def GetObjectInfo(self):
    return self.__object_info