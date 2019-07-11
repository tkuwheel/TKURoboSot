#!/usr/bin/env python
import roslib
import tf
import rospy
import math
import numpy as np
from nubot_common.msg import OminiVisionInfo
from nubot_common.srv import Shoot
from nubot_common.srv import BallHandle
from transfer.msg import PPoint
from vision.msg import Object
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

ROBOT_NUM = 1
BLUE_GOAL = {'x': 300, 'y': 0}
YELLOW_GOAL = {'x': -300, 'y': 0}

handle_pub = rospy.Publisher('/robot1/BallIsHandle', Bool, queue_size=1)
vision_pub = rospy.Publisher('/vision/object', Object, queue_size=1)
location_pub = rospy.Publisher('/akf_pose', PoseWithCovarianceStamped, queue_size=1) 

def OmniVisionCallback(data):
    rx = data.robotinfo[1 -1].pos.x
    ry = data.robotinfo[1 -1].pos.y
    ra = data.robotinfo[1 -1].heading.theta
    q = PoseWithCovarianceStamped()
    q.pose.pose.position.x = rx * 0.01
    q.pose.pose.position.y = ry * 0.01
    qn = tf.transformations.quaternion_from_euler(0, 0, ra)
    q.pose.pose.orientation.x = qn[0]
    q.pose.pose.orientation.y = qn[1]
    q.pose.pose.orientation.z = qn[2]
    q.pose.pose.orientation.w = qn[3]
    location_pub.publish(q)

    m = Object()
    m.ball_dis = data.ballinfo.real_pos.radius
    m.ball_ang = math.degrees(data.ballinfo.real_pos.angle)

    gr_dot = np.dot([BLUE_GOAL['x'] - rx, BLUE_GOAL['y'] - ry], [1, 0])
    gr_cos = gr_dot / math.hypot(BLUE_GOAL['x'] - rx, BLUE_GOAL['y'] - ry)
    gr_sign = np.sign(np.cross([1, 0], [BLUE_GOAL['x'] - rx, BLUE_GOAL['y'] - ry]))
    gr_ang = math.degrees(math.acos(gr_cos)) * gr_sign - math.degrees(ra)
    gr_ang = (gr_ang + 360) % 360
    if gr_ang > 180:
      gr_ang = gr_ang - 360
    m.blue_ang = gr_ang
    m.blue_dis = math.hypot(BLUE_GOAL['x'] - rx, BLUE_GOAL['y'] - ry)
    m.blue_fix_ang = m.blue_ang
    m.blue_fix_dis = m.blue_dis

    gr_dot = np.dot([YELLOW_GOAL['x'] - rx, YELLOW_GOAL['y'] - ry], [1, 0])
    gr_cos = gr_dot / math.hypot(YELLOW_GOAL['x'] - rx, YELLOW_GOAL['y'] - ry)
    gr_sign = np.sign(np.cross([1, 0], [YELLOW_GOAL['x'] - rx, YELLOW_GOAL['y'] - ry]))
    gr_ang = math.degrees(math.acos(gr_cos)) * gr_sign - math.degrees(ra)
    gr_ang = (gr_ang + 360) % 360
    if gr_ang > 180:
      gr_ang = gr_ang - 360
    m.yellow_ang = gr_ang
    m.yellow_dis = math.hypot(YELLOW_GOAL['x'] - rx, YELLOW_GOAL['y'] - ry)
    m.yellow_fix_ang = m.yellow_ang
    m.yellow_fix_dis = m.yellow_dis

    vision_pub.publish(m)

def ShootCallback(data):
    shoot_client(data.data)

def HandleCallback(data):
    r = handle_client()
    handle_pub.publish(r)

def shoot_client(power, pos = 1):
    rospy.wait_for_service('nubot1/Shoot', 1)
    try:
        server = rospy.ServiceProxy('nubot1/Shoot', Shoot)
        resp1 = server(power, pos)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def handle_client():
    rospy.wait_for_service("/nubot1/BallHandle", 1)
    try:
        client = rospy.ServiceProxy("/nubot1/BallHandle", BallHandle)
        resp1 = client(1)
        return resp1.BallIsHolding
    except rospy.ServiceException :
        print ("Service call failed")

def listener():
    rospy.init_node('gazebo_relay', anonymous=True)

    rospy.Subscriber("/motion/shoot", Int32, ShootCallback)
    rospy.Subscriber("/motion/hold", Bool, HandleCallback)
    rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, OmniVisionCallback)

    rospy.spin()

if __name__ == '__main__':
    listener()