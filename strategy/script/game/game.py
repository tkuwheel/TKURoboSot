#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import GameStateConfig

def callback(config, level):
    return config

if __name__ == '__main__':
    rospy.init_node('game', anonymous = False)
    print("Start Game State Config Server")
    try:
        server = DynamicReconfigureServer(GameStateConfig, callback)
        rospy.spin()
    except rospy.ROSInterruptException: pass