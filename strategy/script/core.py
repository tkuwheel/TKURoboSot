#!/usr/bin/env python3
import rospy
import sys
#sys.path.insert(0, "/home/superwheel/RoboSot/devel/lib/python2.7/dist-packages")
import math
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_sys import log, SysCheck, logInOne
from methods.chase import Chase
from methods.attack import strategy as attack
from dynamic_reconfigure.server import Server
from strategy.cfg import GameStateConfig

class Core(Robot, StateMachine):
  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')

  toChase  = idle.to(chase) | attack.to(chase)
  toIdle   = chase.to(idle) | attack.to(idle)
  toAttack = chase.to(attack)

  def on_toChase(self):
    log("To Chase")
  
  def on_toIdle(self):
    log("To Idle")

  def on_toAttack(self):
    log("To Attack")

  def __init__(self, robot_num, sim = False):
    self.CC  = Chase()
    self.sim = sim
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)

class Strategy(object):
  def __init__(self):
    pass

  def Callback(self, config, level):
    rospy.loginfo(config)
    return config

  def main(self, argv):
    rospy.init_node('core', anonymous=True)
    rate = rospy.Rate(10)

    srv = Server(GameStateConfig, self.Callback)
    print(srv)

    if SysCheck(argv) == "Native Mode":
      log("Start Native")
      robot = Core(1)
    elif SysCheck(argv) == "Simulative Mode":
      log("Start Sim")
      robot = Core(1, True)

    while not rospy.is_shutdown():
      gains = rospy.get_param("game_state_server", False)
      targets = robot.GetObjectInfo()

      while targets is not None:
        targets = robot.GetObjectInfo()
        if robot.is_idle:
          print(robot.current_state)
          exit()
        rospy.spinOnce()
        rate.sleep()

if __name__ == '__main__':
  try:
    s = Strategy()
    s.main(sys.argv[1:])
  except rospy.ROSInterruptException:
    pass