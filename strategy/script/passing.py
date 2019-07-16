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
from strategy.cfg import PassingConfig
import dynamic_reconfigure.client

class Core(Robot, StateMachine):

  last_ball_dis = 0
  last_time     = time.time()

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  shoot  = State('Shoot')
  point  = State('Point')
  movement = State('Movement')

  toIdle   = chase.to(idle) |  movement.to(idle) | point.to(idle) | idle.to.itself() | shoot.to(idle)
  toChase  = idle.to(chase) |  chase.to.itself() | movement.to(chase) | point.to(chase)
  toShoot  = movement.to(shoot) | point.to(shoot)
  toMovement = chase.to(movement) | movement.to.itself() | point.to(movement)
  toPoint  = point.to.itself() | idle.to(point) | chase.to(point) | movement.to(point) | shoot.to(point)

  def Callback(self, config, level):
    self.game_level = config['level']
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
    self.BC  = Behavior()
    self.block = 0
    dsrv = DynamicReconfigureServer(PassingConfig, self.Callback)

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

    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos = 1):
    self.RobotShoot(power, pos)

  def on_toMovement(self, tang):
    x, y, yaw, arrived = self.BC.Orbit(tang)
    self.MotionCtrl(x, y, yaw, True)
    return arrived

  def on_toPoint(self, tx, ty, tyaw):
    x, y, yaw, arrived = self.BC.Go2Point(tx, ty, tyaw)

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

  can_shoot = False
  current_index = 0
  current_point = [0, 0, 0]
  padding_ball  = 35
  padding_target= 80
  ball1 = (-115 + padding_ball, 120)
  ball2 = (-115 + padding_ball, 40)
  ball3 = (-115 + padding_ball, -40)
  ball4 = (-115 + padding_ball, -120)
  target1 = (115 - padding_target, 120)
  target2 = (115 - padding_target, 40)
  target3 = (115 - padding_target, -40)
  target4 = (115 - padding_target, -120)
  level1 = {'balls_point': [ball1], 'targets_point': [target1]}
  level2 = {'balls_point': [ball1, ball4], 'targets_point': [target4, target1]}
  level3 = {'balls_point': [ball1, ball2, ball4], 'targets_point': [target4, target2, target1]}
  level4 = {'balls_point': [ball1, ball2, ball3, ball4], 'targets_point': [target3, target4, target1, target2]}

  def __init__(self, num, sim=False):
    rospy.init_node('passing', anonymous=True)
    self.rate = rospy.Rate(1000)
    self.robot = Core(num, sim)
    self.dclient = dynamic_reconfigure.client.Client("passing", timeout=30, config_callback=None)
    self.main()

  def ToChase(self):
    mode = self.robot.strategy_mode
    if mode == "Defense":
      self.robot.toChase("Classic")
    elif mode == "Attack":
      pass
    else: 
      log("Unknown Chase Mode")

  def ToMovement(self):
    mode = self.robot.strategy_mode
    if mode == "Attack":
      pass

  def UpdateCurrentPoint(self, x, y, yaw):
    Strategy.current_point[0] = x
    Strategy.current_point[1] = y
    Strategy.current_point[2] = yaw

  def main(self):
    while not rospy.is_shutdown():

      self.robot.PubCurrentState()

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()

      if self.robot.game_level == "Level1":
        level = self.level1
      elif self.robot.game_level == "Level2":
        level = self.level2
      elif self.robot.game_level == "Level3":
        level = self.level3
      elif self.robot.game_level == "Level4":
        level = self.level4

      if not self.robot.is_idle and not self.robot.game_start:
        self.robot.toIdle()

      if self.robot.is_idle:
        if self.robot.game_start:
          Strategy.current_index = 0
          Strategy.can_shoot = False
          self.UpdateCurrentPoint(level['balls_point'][Strategy.current_index][0], level['balls_point'][Strategy.current_index][1], 180)
          self.robot.toPoint(Strategy.current_point[0], Strategy.current_point[1], Strategy.current_point[2])

      if self.robot.is_point:
        if self.robot.toPoint(Strategy.current_point[0], Strategy.current_point[1], Strategy.current_point[2]):
          if self.robot.CheckBallHandle():
            if Strategy.can_shoot:
              self.robot.toShoot(80)
            else:
              self.UpdateCurrentPoint(level['targets_point'][Strategy.current_index][0], level['targets_point'][Strategy.current_index][1], 0)
              self.robot.toMovement(0 - position['location']['yaw'])
          else:
            self.robot.toChase("Straight")
        else:
          self.robot.toPoint(Strategy.current_point[0], Strategy.current_point[1], Strategy.current_point[2])

      if self.robot.is_chase:
        if self.robot.CheckBallHandle():
          self.UpdateCurrentPoint(0, 0, 90)
          self.robot.toPoint(Strategy.current_point[0], Strategy.current_point[1], Strategy.current_point[2])
          # self.robot.toMovement(Strategy.current_point[2] - position['location']['yaw'])
        else:
          self.robot.toChase("Straight")

      if self.robot.is_movement:
        if not self.robot.CheckBallHandle():
          self.robot.toChase("Straight")
        elif self.robot.toMovement(0 - position['location']['yaw']):
          self.robot.toPoint(Strategy.current_point[0], Strategy.current_point[1], Strategy.current_point[2])
          Strategy.can_shoot = True
        else:
          self.robot.toMovement(0 - position['location']['yaw'])

      if self.robot.is_shoot:
        if Strategy.current_index + 1 >= len(level['balls_point']):
          print("Endgame")
          self.dclient.update_configuration({"game_start": False})
          self.robot.toIdle()
        else:
          Strategy.current_index += 1
          Strategy.can_shoot = False
          self.UpdateCurrentPoint(level['balls_point'][Strategy.current_index][0], level['balls_point'][Strategy.current_index][1], 180)
          self.robot.toPoint(Strategy.current_point[0], Strategy.current_point[1], Strategy.current_point[2])

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
