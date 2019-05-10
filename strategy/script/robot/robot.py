import rospy
import math
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import Shoot
from nubot_common.srv import BallHandle
from sensor_msgs.msg import JointState
from vision.msg import Object
from transfer.msg import PPoint
from std_msgs.msg import String

SIM_VISION_TOPIC = "nubot{}/omnivision/OmniVisionInfo"
SIM_CMDVEL_TOPIC = "nubot{}/nubotcontrol/velcmd"
SIM_SHOOT_SRV  = "nubot{}/Shoot"
SIM_HANDLE_SRV = "nubot{}/Ballhandle"
VISION_TOPIC   = "vision/object"

STRATEGY_STATE_TOPIC = "robot{}/strategy/state"

class Robot(object):

  __robot_info = {'location' : None}
  __object_info = {'ball':{'dis' : 0, 'ang' : 0},
                   'Cyan':{'dis' : 0, 'ang' : 0},
                   'Magenta':{'dis' : 0, 'ang' : 0},
                   'velocity' : 0 }

  def ShowRobotInfo(self):
    print("Robot informations: {}".format(self.__robot_info))
    print("Objects informations: {}".format(self.__object_info))

  def __init__(self, robot_num, sim = False):
    self.robot_number = robot_num

    if not sim :
      rospy.Subscriber("vision/object", Object, self._GetVision)   
    else:
      self._Subscriber(SIM_VISION_TOPIC.format(self.robot_number))
      self.cmdvel_pub = self._Publisher(SIM_CMDVEL_TOPIC.format(self.robot_number), VelCmd)
      self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC.format(self.robot_number), String)

  def _Subscriber(self, topic):
    rospy.Subscriber(topic.format(self.robot_number), \
                      OminiVisionInfo, \
                      self._GetOmniVsison)
    rospy.Subscriber((topic + "/GoalInfo").format(self.robot_number), \
                      PPoint, \
                      self._GetGoalInfo)

  def _Publisher(self, topic, mtype):
    return rospy.Publisher(topic.format(self.robot_number), mtype, queue_size=1)

  def _GetOmniVsison(self, vision):
    self.__object_info['ball']['dis'] = vision.ballinfo.real_pos.radius
    self.__object_info['ball']['ang'] = math.degrees(vision.ballinfo.real_pos.angle)


  def _GetVision(self, vision):
    self.__object_info['ball']['dis'] = vision.ball_dis
    self.__object_info['ball']['ang'] = math.degrees(vision.ball_ang)
    self.__object_info['Cyan']['dis'] = vision.blue_dis
    self.__object_info['Cyan']['ang'] = vision.blue_ang
    self.__object_info['Magenta']['dis'] = vision.yellow_dis
    self.__object_info['Magenta']['ang'] = vision.yellow_ang
    
  def _GetGoalInfo(self, goal_info):
    self.__object_info['Cyan']['dis'] = goal_info.left_radius
    self.__object_info['Cyan']['ang'] = goal_info.left_angle
    self.__object_info['Magenta']['dis'] = goal_info.right_radius
    self.__object_info['Magenta']['ang'] = goal_info.right_angle

  '''def RobotStatePub(self, state):
    s = String()
    s.data = state
    self.state_pub.publish(s)'''

  def RobotCtrl(self, x, y, yaw):
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

  def RobotShoot(self, power, pos) :
    rospy.wait_for_service(SIM_SHOOT_SRV.format(self.robot_number))
    try:
      client = rospy.ServiceProxy(SIM_SHOOT_SRV.format(self.robot_number), Shoot)
      resp1 = client(power, pos)
      return resp1
    except rospy.ServiceException :
      print ("Service call failed")

  def RobotBallhandle(self):
    rospy.wait_for_service(SIM_HANDLE_SRV.format(self.robot_number))
    try:
      client = rospy.ServiceProxy(SIM_HANDLE_SRV.format(self.robot_number), BallHandle)
      resp1 = client(1)
      return resp1
    except rospy.ServiceException :
      print ("Service call failed")
  
