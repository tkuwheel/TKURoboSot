#!/usr/bin/env python3

import rospy
import math
from transfer.model import my_math
from transfer.msg import PPoint
from gazebo_msgs.msg import ModelStates

class ModelTransfer(object):

    def __init__(self):
        self.loadParam()
        self.publisher()        
        self.subscriber()

    def loadParam(self):
        if rospy.has_param('/cyan/num'):
            self.my_robot_num = rospy.get_param('/cyan/num')
        if rospy.has_param('/magenta/num'):
            self.oppo_robot_num = rospy.get_param('/magenta/num')

    def subscriber(self):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.getModel)

    def publisher(self):
        if self.my_robot_num <= 0:
            raise('Robot number error')
            exit(1)
        if self.my_robot_num >= 1:
            self.nubot1_goal_pub = rospy.Publisher('/nubot1/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 2:
            self.nubot2_goal_pub = rospy.Publisher('/nubot2/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 3:
            self.nubot3_goal_pub = rospy.Publisher('/nubot3/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 4:
            self.nubot4_goal_pub = rospy.Publisher('/nubot4/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)
        if self.my_robot_num >= 5:
            self.nubot5_goal_pub = rospy.Publisher('/nubot5/omnivision/OmniVisionInfo/GoalInfo', PPoint, queue_size=100)

    def getModel(self, models):

        for index, name in enumerate(models.name):
            if name == 'left_goal':
                left_goal_x = models.pose[index].position.x
                left_goal_y = models.pose[index].position.y
            elif name == 'right_goal':
                right_goal_x = models.pose[index].position.x
                right_goal_y = models.pose[index].position.y
            elif name == 'nubot1':
                nubot1_x = models.pose[index].position.x
                nubot1_y = models.pose[index].position.y
                _, _, nubot1_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot2':
                nubot2_x= models.pose[index].position.x
                nubot2_y = models.pose[index].position.y
                _, _, nubot2_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot3':
                nubot3_x= models.pose[index].position.x
                nubot3_y = models.pose[index].position.y
                _, _, nubot3_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot4':
                nubot4_x= models.pose[index].position.x
                nubot4_y = models.pose[index].position.y
                _, _, nubot4_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            elif name == 'nubot5':
                nubot5_x= models.pose[index].position.x
                nubot5_y = models.pose[index].position.y
                _, _, nubot5_yaw = my_math.quaternionTEuler(
                        x=models.pose[index].orientation.x,
                        y=models.pose[index].orientation.y,
                        z=models.pose[index].orientation.z,
                        w=models.pose[index].orientation.w,)
            else:
                pass
        # rospy.loginfo('robot num: {}'.format(self.my_robot_num))
        if self.my_robot_num <= 0:
            raise('Robot number error')
            exit(1)
        if len(models.name) == 5 + self.my_robot_num + self.oppo_robot_num:
            if self.my_robot_num >= 1:
                nubot1_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot1_x, y=left_goal_y-nubot1_y,
                        remainder=3)
                nubot1_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot1_x, y=left_goal_y-nubot1_y, 
                        yaw=nubot1_yaw, remainder=3)
                nubot1_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot1_x, y=right_goal_y-nubot1_y,
                        remainder=3)
                nubot1_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot1_x, y=right_goal_y-nubot1_y, 
                        yaw=nubot1_yaw, remainder=3)
                nubot1_goal = PPoint()
                nubot1_goal.left_angle = nubot1_left_goal_ang
                nubot1_goal.left_radius = nubot1_left_goal_dis
                nubot1_goal.right_angle = nubot1_right_goal_ang
                nubot1_goal.right_radius = nubot1_right_goal_dis
                self.nubot1_goal_pub.publish(nubot1_goal)
            if self.my_robot_num >= 2:
                nubot2_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot2_x, y=left_goal_y-nubot2_y,
                        remainder=3)
                nubot2_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot2_x, y=left_goal_y-nubot2_y, 
                        yaw=nubot2_yaw, remainder=3)
                nubot2_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot2_x, y=right_goal_y-nubot2_y,
                        remainder=3)
                nubot2_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot2_x, y=right_goal_y-nubot2_y, 
                        yaw=nubot2_yaw, remainder=3)
                nubot2_goal = PPoint()
                nubot2_goal.left_angle = nubot2_left_goal_ang
                nubot2_goal.left_radius = nubot2_left_goal_dis
                nubot2_goal.right_angle = nubot2_right_goal_ang
                nubot2_goal.right_radius = nubot2_right_goal_dis
                self.nubot2_goal_pub.publish(nubot2_goal)
            if self.my_robot_num >= 3:
                nubot3_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot3_x, y=left_goal_y-nubot3_y,
                        remainder=3)
                nubot3_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot3_x, y=left_goal_y-nubot3_y, 
                        yaw=nubot3_yaw, remainder=3)
                nubot3_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot3_x, y=right_goal_y-nubot3_y,
                        remainder=3)
                nubot3_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot3_x, y=right_goal_y-nubot3_y, 
                        yaw=nubot3_yaw, remainder=3)
                nubot3_goal = PPoint()
                nubot3_goal.left_angle = nubot3_left_goal_ang
                nubot3_goal.left_radius = nubot3_left_goal_dis
                nubot3_goal.right_angle = nubot3_right_goal_ang
                nubot3_goal.right_radius = nubot3_right_goal_dis
                self.nubot3_goal_pub.publish(nubot3_goal)
            if self.my_robot_num >= 4:
                nubot4_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot4_x, y=left_goal_y-nubot4_y,
                        remainder=3)
                nubot4_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot4_x, y=left_goal_y-nubot4_y, 
                        yaw=nubot4_yaw, remainder=3)
                nubot4_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot4_x, y=right_goal_y-nubot4_y,
                        remainder=3)
                nubot4_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot4_x, y=right_goal_y-nubot4_y, 
                        yaw=nubot4_yaw, remainder=3)
                nubot4_goal = PPoint()
                nubot4_goal.left_angle = nubot4_left_goal_ang
                nubot4_goal.left_radius = nubot4_left_goal_dis
                nubot4_goal.right_angle = nubot4_right_goal_ang
                nubot4_goal.right_radius = nubot4_right_goal_dis
                self.nubot4_goal_pub.publish(nubot4_goal)
            if self.my_robot_num >= 5:
                nubot5_left_goal_dis = my_math.calDis(
                        x=left_goal_x-nubot4_x, y=left_goal_y-nubot5_y,
                        remainder=3)
                nubot5_left_goal_ang = my_math.calAng(
                        x=left_goal_x-nubot4_x, y=left_goal_y-nubot5_y, 
                        yaw=nubot5_yaw, remainder=3)
                nubot5_right_goal_dis = my_math.calDis(
                        x=right_goal_x-nubot4_x, y=right_goal_y-nubot5_y,
                        remainder=3)
                nubot5_right_goal_ang = my_math.calAng(
                        x=right_goal_x-nubot4_x, y=right_goal_y-nubot5_y, 
                        yaw=nubot5_yaw, remainder=3)
                nubot5_goal = PPoint()
                nubot5_goal.left_angle = nubot5_left_goal_ang
                nubot5_goal.left_radius = nubot5_left_goal_dis
                nubot5_goal.right_angle = nubot5_right_goal_ang
                nubot5_goal.right_radius = nubot5_right_goal_dis
                self.nubot5_goal_pub.publish(nubot5_goal)
    
if __name__ == '__main__':
    rospy.init_node('transfer', anonymous=True)
    rate = rospy.Rate(50)
    model_transfer = ModelTransfer()
    while not rospy.is_shutdown():
        rate.sleep()
