#!/usr/bin/env python
import rospy
import sys
import math
import time
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_sys import log, SysCheck, logInOne
from methods.chase import Chase
from methods.attack import Attack
from methods.behavior import Behavior
from dynamic_reconfigure.server import Server
from strategy.cfg import GameStateConfig

class Core(Robot, StateMachine):
  def __init__(self, robot_num, sim = False):
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.AC  = Attack()
    self.BC  = Behavior()
    self.sim = sim

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')
  shoot  = State('Shoot')
  orbit  = State('Orbit')
  point  = State('Point')

  toIdle   = chase.to(idle) | attack.to(idle)  | orbit.to(idle) | point.to(idle)
  toChase  = idle.to(chase) | attack.to(chase) | chase.to.itself() | orbit.to(chase)
  toAttack = chase.to(attack) | attack.to.itself() | shoot.to(attack) | orbit.to(attack)
  toShoot  = attack.to(shoot)
  toOrbit  = chase.to(orbit) | orbit.to.itself()
  toPoint  = point.to.itself() | idle.to(point)

  def on_toIdle(self):
    for i in range(0,100):
        self.MotionCtrl(0,0,0)
    log("To Idle")

  def on_toChase(self, t, side, method = "Classic"):
    if method == "Classic":
      x, y, yaw = self.CC.ClassicRounding(t[side]['ang'],\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)

      if self.RobotBallHandle():
        self.toAttack(t, side)
      else:
        pass

    elif method == "Straight":
      x, y, yaw = self.CC.StraightForward(t['ball']['dis'], t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)

  def on_toAttack(self, t, side):
    x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos):
    self.RobotShoot(power, pos)

  def on_toOrbit(self, t, side):
    x, y, yaw = self.CC.Orbit(t[side]['ang'])
    self.MotionCtrl(x, y, yaw, True)

  def on_toPoint(self, tx, ty, tyaw):
    x, y, yaw = self.BC.Go2Point(tx, ty, tyaw)
    self.MotionCtrl(x, y, yaw)

  def PubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

  def CheckBallHandle(self):
    return self.RobotBallHandle()

class Strategy(object):
  def __init__(self):
    gains = rospy.get_param("/core")
    self.game_start = gains['game_start']
    self.game_state = gains['game_state']
    self.side       = gains['side']
    self.opp_side   = 'Cyan' if gains['side'] == 'Magenta' else 'Magenta'

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.side       = config['side']
    self.opp_side   = 'Cyan' if config['side'] == 'Magenta' else 'Magenta'
    return config

  def main(self, argv):
    rospy.init_node('core', anonymous=True)
    rate = rospy.Rate(1000)

    dsrv = Server(GameStateConfig, self.Callback)

    TEST_MODE = False
    if SysCheck(argv) == "Native Mode":
      log("Start Native")
      robot = Core(1)
    elif SysCheck(argv) == "Simulative Mode":
      log("Start Sim")
      robot = Core(1, True)
    elif SysCheck(argv) == "Test Mode":
      log("Test Mode")
      robot = Core(1, True)
      TEST_MODE = True

    while not rospy.is_shutdown():

      robot.PubCurrentState()
      targets = robot.GetObjectInfo()

      if targets is not None and not TEST_MODE:
        if not robot.is_idle and not self.game_start:
          robot.toIdle()
        elif robot.is_idle and self.game_start:
          robot.toChase(targets, self.side)
        elif robot.is_chase:
          robot.toChase(targets, self.side)

        if robot.is_chase and abs(targets['ball']['ang']) <= 20 \
                          and targets['ball']['dis'] <= 50:
          robot.toAttack(targets, self.side)
        elif robot.is_attack:
          robot.toAttack(targets, self.side)

        if robot.is_attack and abs(targets['ball']['ang']) > 20 \
                           and targets['ball']['dis'] > 50:
          robot.toChase(targets, self.side)

        if robot.is_attack and abs(targets[self.side]['ang']) < 10:
          pass
          # robot.toShoot(3, 1)

        if robot.is_shoot:
          robot.toAttack(targets, self.side)

      ### Test Mode ###
      else:
        Points = [{'x':0, 'y':0, 'yaw':0}, \
                  {'x':100, 'y':100, 'yaw':90}, \
                  {'x':-100, 'y':-100, 'yaw':-90}]
        if not robot.is_idle and not self.game_start:
          robot.toIdle()
        elif robot.is_idle and self.game_start:
          # robot.toChase(targets, self.side, "Straight")
          robot.toPoint(Points[0]['x'], Points[0]['y'], Points[0]['yaw'])
        elif robot.is_chase:
          robot.toChase(targets, self.side, "Straight")
        elif robot.is_point:
          robot.toPoint(Points[0]['x'], Points[0]['y'], Points[0]['yaw'])

        if robot.is_chase and abs(targets['ball']['ang']) <= 20 \
                          and targets['ball']['dis'] <= 42:
          robot.toOrbit(targets, self.side)
        elif robot.is_orbit:
          robot.toOrbit(targets, self.side)

        if robot.is_orbit and abs(targets['ball']['ang']) > 20 \
                          and targets['ball']['dis'] > 42:
          robot.toChase(targets, self.side, "Straight")

        if robot.is_orbit and abs(targets[self.side]['ang'] - targets['ball']['ang']) <= 10 \
                          and robot.RobotBallHandle():
          robot.toAttack(targets, self.side)
        elif robot.is_attack:
          robot.toAttack(targets, self.side)

        if robot.is_attack and abs(targets['ball']['ang']) > 20 \
                           and targets['ball']['dis'] > 50:
          robot.toChase(targets, self.side)

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