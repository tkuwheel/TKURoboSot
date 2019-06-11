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
from strategy.cfg import StrategyConfig

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
    log("To Idle1")

  def on_toChase(self, t, side, method = "Classic"):
    if method == "Classic":
      x, y, yaw = self.CC.ClassicRounding(t[side]['ang'],\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)
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

class Strategy(Robot):
  def __init__(self):
    self.game_start = False
    self.game_state = "Kick_Off"
    self.side       = "Yellow"
    self.opp_side   = 'Yellow' if self.side == 'Blue' else 'Blue'

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.side       = config['our_goal']
    self.opp_side   = 'Yellow' if config['our_goal'] == 'Blue' else 'Blue'

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])
    return config

  def main(self, argv):
    rospy.init_node('core', anonymous=True)
    rate = rospy.Rate(1000)

    dsrv = Server(StrategyConfig, self.Callback)

    TEST_MODE = True
    if SysCheck(argv) == "Native Mode":
      
      log("Start Native")
      robot = Core(1)
      
    elif SysCheck(argv) == "Simulative Mode":
      log("Start Sim")
      robot = Core(1, True)

    while not rospy.is_shutdown():

      robot.PubCurrentState()
      
      targets = robot.GetObjectInfo()

      if targets is None or targets['ball']['ang'] is 999: # Can not find ball
        robot.toIdle()
      else:
        if not robot.is_idle and not self.game_start:
          robot.toIdle()
        elif robot.is_idle and self.game_start:
          robot.toChase(targets, self.opp_side)
        elif robot.is_chase:
          robot.toChase(targets, self.opp_side)

        if robot.is_chase and robot.CheckBallHandle():
          robot.toAttack(targets, self.opp_side)
        elif robot.is_attack:
          robot.toAttack(targets, self.opp_side)

        if robot.is_attack and not robot.CheckBallHandle():
          robot.toChase(targets, self.opp_side)

        if robot.is_attack and abs(targets[self.opp_side]['ang']) < 10:
          robot.toShoot(3, 1)

        if robot.is_shoot:
          robot.toAttack(targets, self.opp_side)

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
