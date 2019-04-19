#!/usr/bin/env python3
import rospy
import sys
#sys.path.insert(0, "/home/superwheel/RoboSot/devel/lib/python2.7/dist-packages")
import math
import time
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_sys import log, SysCheck, logInOne
from methods.chase import Chase
from methods.attack import Attack
from dynamic_reconfigure.server import Server
from strategy.cfg import GameStateConfig

class Core(Robot, StateMachine):
  def __init__(self, robot_num, sim = False):
    self.CC  = Chase()
    self.AC  = Attack()
    self.sim = sim
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')

  toChase  = idle.to(chase) | attack.to(chase) | chase.to(chase)
  toIdle   = chase.to(idle) | attack.to(idle)
  toAttack = chase.to(attack)

  def on_toChase(self, t):
    o = self.CC.ClassicRounding(t['magenta_goal']['ang'],\
                                t['ball']['dis'],\
                                t['ball']['ang'])
    self.RobotCtrl(o['v_x'], o['v_y'], o['v_yaw'])

  def on_toIdle(self):
    log("To Idle")

  def on_toAttack(self, t):
    o = self.AC.ClassicAttacking(t['magenta_goal']['dis'], t['magenta_goal']['ang'])
    self.RobotCtrl(o['v_x'], o['v_y'], o['v_yaw'])

  def pubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

class Strategy(object):
  def __init__(self):
    gains = rospy.get_param("/core")
    self.game_start = gains['game_start']
    self.game_state = gains['game_state']
    self.side       = gains['side']

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.side       = config['side']
    return config

  def main(self, argv):
    rospy.init_node('core', anonymous=True)
    rate = rospy.Rate(1000)

    dsrv = Server(GameStateConfig, self.Callback)

    if SysCheck(argv) == "Native Mode":
      log("Start Native")
      robot = Core(1)
    elif SysCheck(argv) == "Simulative Mode":
      log("Start Sim")
      robot = Core(1, True)

    while not rospy.is_shutdown():

      robot.pubCurrentState()
      targets = robot.GetObjectInfo()

      if targets is not None:
        if not robot.is_idle and not self.game_start:
          # stay idle
          robot.toIdle()
        elif robot.is_idle and self.game_start:
          # go chase
          robot.toChase(targets)
        elif robot.is_chase and targets['ball']['dis'] >= 37:
          # keep chase
          # log("keep{}".format(targets['ball']['dis']))
          robot.toChase(targets)
        elif robot.is_chase and targets['ball']['ang'] < 37:
          # go attack
          robot.toAttack(targets)
        elif robot.is_attack and targets['ball']['dis'] > 30:
          # back chase
          robot.toChase(targets)

      if rospy.is_shutdown():
        log('shutdown')
        break

      rate.sleep()

if __name__ == '__main__':
  try:
    s = Strategy()
    s.main(sys.argv[1:])
  except rospy.ROSInterruptException:
    pass