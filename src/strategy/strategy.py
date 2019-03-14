#!/usr/bin/env python3
import rospy
import math
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from transfer.msg import PPoint

class Strategy(object):
    def __init__(self, robot_number):
        self.robot_number = robot_number
        self.init_flag1 = 0
        self.init_flag2 = 0
        self.subscriber()
        self.publisher()

    def strategy(self):
        self.init_flag = self.init_flag1 and self.init_flag2
        if self.init_flag:
            alpha = math.radians(self.ball_ang - self.right_goal_ang)
            beta = 0.7
            if abs(alpha) > beta:
                angle_type = 'beta'
                if alpha > 0:
                    alpha = beta
                else:
                    alpha = -beta
            else:
                angle_type = 'alpha'
            br_x = self.ball_dis * math.cos(math.radians(self.ball_ang))
            br_y = self.ball_dis * math.sin(math.radians(self.ball_ang))

            if self.ball_dis <= 50 and self.ball_ang <= 20:
                v_x = self.right_goal_dis * math.cos(math.radians(self.right_goal_ang))
                v_y = self.right_goal_dis * math.sin(math.radians(self.right_goal_ang))
                v_yaw = self.right_goal_ang
                strategy_type = 'attack'
            else:
                v_x = br_x * math.cos(alpha) - br_y * math.sin(alpha)
                v_y = br_x * math.sin(alpha) + br_y * math.cos(alpha)
                v_yaw = self.right_goal_ang
                strategy_type = 'chase'
            
            print('\rangle_type: {}, strategy_type: {}'.format(angle_type, strategy_type), end='')
            self.pubNubotCtrl(v_x, v_y, v_yaw)

        else:
            print('unready')

    def subscriber(self):
        rospy.Subscriber('nubot{}/omnivision/OmniVisionInfo'.format(self.robot_number), OminiVisionInfo, self.getOmniVsison)
        rospy.Subscriber('nubot{}/omnivision/OmniVisionInfo/GoalInfo'.format(self.robot_number), PPoint, self.getGoalInfo)

    def publisher(self):
        self.nubot_cmd_pub = rospy.Publisher('nubot{}/nubotcontrol/velcmd'.format(self.robot_number), VelCmd, queue_size=100)

    def getOmniVsison(self, vision):
        self.init_flag1 = 1
        self.ball_dis = vision.ballinfo.real_pos.radius
        self.ball_ang = math.degrees(vision.ballinfo.real_pos.angle)
    
    def getGoalInfo(self, goal_info):
        self.init_flag2 = 1
        self.left_goal_dis = goal_info.left_radius
        self.left_goal_ang = goal_info.left_angle
        self.right_goal_dis = goal_info.right_radius
        self.right_goal_ang = goal_info.right_angle

    def pubNubotCtrl(self, v_x, v_y, v_yaw):
        vel = VelCmd()
        vel.Vx = v_x/5
        vel.Vy = v_y/5
        vel.w = v_yaw/18
        if abs(vel.w) >= 10:
            if vel.w < 0:
                vel.w = -10
            else:
                vel.w = 10
        elif abs(vel.w) <= 1:
            if vel.w < 0:
                vel.w = -0.1
            else:
                vel.w = 0.1
        else:
            pass

        self.nubot_cmd_pub.publish(vel)

if __name__ == '__main__':
    
    rospy.init_node('strategy', anonymous=True)
    rate = rospy.Rate(50)
    robot = Strategy(1)
    while not rospy.is_shutdown():
        robot.strategy()
        rate.sleep()
    