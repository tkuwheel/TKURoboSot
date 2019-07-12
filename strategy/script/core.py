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
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import RobotConfig
import dynamic_reconfigure.client

class Core(Robot, StateMachine):

  last_ball_dis = 0
  last_time     = time.time()

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')
  shoot  = State('Shoot')
  point  = State('Point')
  movement = State('Movement')

  toIdle   = chase.to(idle) | attack.to(idle)  | movement.to(idle) | point.to(idle) | idle.to.itself()
  toChase  = idle.to(chase) | attack.to(chase) | chase.to.itself() | movement.to(chase) | point.to(chase)
  toAttack = chase.to(attack) | attack.to.itself() | shoot.to(attack) | movement.to(attack)
  toShoot  = attack.to(shoot)
  toMovement = chase.to(movement) | movement.to.itself()
  toPoint  = point.to.itself() | idle.to(point)

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.run_point  = config['run_point']
    self.our_side   = config['our_side']
    self.opp_side   = 'Blue' if self.our_side == 'Yellow' else 'Yellow'
    self.run_x      = config['run_x']
    self.run_y      = config['run_y']
    self.run_yaw    = config['run_yaw']
    self.strategy_mode = config['strategy_mode']
    self.maximum_v = config['maximum_v']

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    return config

  def __init__(self, robot_num, sim = False):
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.AC  = Attack()
    self.BC  = Behavior()
    self.block = 0
    dsrv = DynamicReconfigureServer(RobotConfig, self.Callback)

  def on_toIdle(self):
    self.goal_dis = 0
    for i in range(0, 10):
        self.MotionCtrl(0,0,0)
    log("To Idle1")

  def on_toChase(self, method = "Classic"):
    t = self.GetObjectInfo()
    side = self.opp_side
    if method == "Classic":
      x, y, yaw = self.CC.ClassicRounding(t[side]['ang'],\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
    elif method == "Straight":
      x, y, yaw = self.CC.StraightForward(t['ball']['dis'], t['ball']['ang'])

    self.Accelerator(80)
    self.MotionCtrl(x, y, yaw)

  def on_toAttack(self, method = "Classic"):
    t = self.GetObjectInfo()
    side = self.opp_side
    if method == "Classic":
      x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
      self.MotionCtrl(x, y, yaw)
    # elif method == "Cross_Over":
    #   x, y, yaw, _ = self.AC.cross_over(t, side, run)
    #   self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos = 1):
    self.RobotShoot(power, pos)

  def on_toMovement(self, method):
    t = self.GetObjectInfo()
    side = self.opp_side
    if method == "Orbit":
      x, y, yaw = self.BC.Orbit(t[side]['ang'])
    self.MotionCtrl(x, y, yaw, True)

  def on_toPoint(self):
    if self.game_state == "Kick_Off" and self.our_side == "Yellow" :
      x, y, yaw, arrived = self.BC.Go2Point(-60, 0, 0)
    elif self.game_state == "Kick_Off" and self.our_side == "Blue" :
      x, y, yaw, arrived = self.BC.Go2Point(60, 0, 180)
    elif self.game_state == "Free_Kick" :
      x, y, yaw, arrived = self.BC.Go2Point(100, 100, 90)
    elif self.game_state == "Free_Ball" :
      x, y, yaw, arrived = self.BC.Go2Point(-100, -100, 180)
    elif self.game_state == "Throw_In" :
      x, y, yaw, arrived = self.BC.Go2Point(-100, -100, 270)
    elif self.game_state == "Coner_Kick":
      x, y, yaw, arrived = self.BC.Go2Point(300, 200, 45)
    elif self.game_state == "Penalty_Kick" :
      x, y, yaw, arrived = self.BC.Go2Point(-100, 100, 135)
    elif self.game_state == "Run_Specific_Point" :
      x, y, yaw, arrived = self.BC.Go2Point(self.run_x, self.run_y, self.run_yaw)
    else:
      log("Unknown Game State")

    self.MotionCtrl(x, y, yaw)
    return arrived

  def PubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

  def CheckBallHandle(self):
    if self.RobotBallHandle():
      ## Back to normal from Accelerator
      self.ChangeVelocityRange(0, self.maximum_v)
      Core.last_ball_dis = 0

    return self.RobotBallHandle()

  def Accelerator(self, exceed = 100):
    t = self.GetObjectInfo()
    if Core.last_ball_dis == 0:
      Core.last_time = time.time()
      Core.last_ball_dis = t['ball']['dis']
    elif t['ball']['dis'] >= Core.last_ball_dis:
      if time.time() - Core.last_time >= 0.8:
        self.ChangeVelocityRange(0, exceed)
    else:
      Core.last_time = time.time()
      Core.last_ball_dis = t['ball']['dis']

class Strategy(object):

  def __init__(self, num, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(1000)
    self.robot = Core(num, sim)
    self.dclient = dynamic_reconfigure.client.Client("core", timeout=30, config_callback=None)
    self.main()

  def RunStatePoint(self):
    if self.robot.toPoint():
      self.dclient.update_configuration({"run_point": False})
      self.robot.toIdle()

  def ToChase(self):
    mode = self.robot.strategy_mode
    if mode == "Defense":
      self.robot.toChase("Classic")
    elif mode == "Attack":
      if self.robot.CheckBallHandle():
        self.ToMovement()
      else:
        self.robot.toChase("Straight")
    else: 
      log("Unknown Chase Mode")

  def ToAttack(self):
    mode = self.robot.strategy_mode
    if mode == "Attack":
      self.robot.toAttack("Classic")
    if mode == "Defense":
      self.robot.toAttack("Classic")
    elif mode == "cross_over":
      self.robot.toAttack("Cross_Over")

  def ToMovement(self):
    mode = self.robot.strategy_mode
    if mode == "Attack":
      self.robot.toMovement('Orbit')

  def main(self):
    while not rospy.is_shutdown():

      self.robot.PubCurrentState()

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()

      # Can not find ball when starting
      if targets is None or targets['ball']['ang'] == 999 and self.robot.game_start:
        print("Can not find ball")
        self.robot.toIdle()
      else:
        if not self.robot.is_idle and not self.robot.run_point and not self.robot.game_start:
          self.robot.toIdle()

        if self.robot.is_idle:
          if self.robot.game_start:
            self.ToChase()
          elif self.robot.run_point:
            self.RunStatePoint()

        if self.robot.is_chase:
          if self.robot.CheckBallHandle():
            self.ToAttack()
          else:
            self.ToChase()

        if self.robot.is_movement:
          if abs(targets[self.robot.opp_side]['ang']) < 10:
            self.ToAttack()
          elif not self.robot.CheckBallHandle():
            self.ToChase()
          else:
            self.ToMovement()

        if self.robot.is_attack:
          if not self.robot.CheckBallHandle():
            self.ToChase()
          elif  abs(targets[self.robot.opp_side]['ang']) < 3:
            self.robot.toShoot(100)
          else:
            self.ToAttack()

        if self.robot.is_shoot:
          self.ToAttack()

      ## Run point
      if self.robot.is_point:
        self.RunStatePoint()

      if rospy.is_shutdown():
        log('shutdown')
        break

      self.rate.sleep()

if __name__ == '__main__':
  try:
    if SysCheck(sys.argv[1:]) == "Native Mode":
      log("Start Native")
      s = Strategy(1, False)
    elif SysCheck(sys.argv[1:]) == "Simulative Mode":
      log("Start Sim")  
      s = Strategy(1, True)
  except rospy.ROSInterruptException:
    pass
