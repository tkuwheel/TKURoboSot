#!/usr/bin/env python

from std_srvs.srv import *
import rospy

def handle_add_two_ints(req):
    print "Got it"
    resp = TriggerResponse()
    resp.success = True
    resp.message = "Got it"
    return resp

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', Trigger, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
