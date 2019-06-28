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
import dynamic_reconfigure.client

class Core(Robot, StateMachine):
  def __init__(self, robot_num, sim = False):
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.AC  = Attack()
    self.BC  = Behavior()
    self.sim = sim
    self.goal_dis = 0
    self.tStart = time.time()

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')
  shoot  = State('Shoot')
  orbit  = State('Orbit')
  point  = State('Point')

  toIdle   = chase.to(idle) | attack.to(idle)  | orbit.to(idle) | point.to(idle) | idle.to.itself()
  toChase  = idle.to(chase) | attack.to(chase) | chase.to.itself() | orbit.to(chase) | point.to(chase)
  toAttack = chase.to(attack) | attack.to.itself() | shoot.to(attack) | orbit.to(attack)
  toShoot  = attack.to(shoot)
  toOrbit  = chase.to(orbit) | orbit.to.itself()
  toPoint  = point.to.itself() | idle.to(point)

  def on_toIdle(self):
    self.goal_dis = 0
    for i in range(0, 10):
        self.MotionCtrl(0,0,0)
    log("To Idle1")

  def on_toChase(self, t, side, method = "Classic"):
     
    
    if method == "Classic":
      x, y, yaw = self.CC.ClassicRounding(t[side]['ang'],\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
      
    elif method == "Straight":
      x, y, yaw = self.CC.StraightForward(t['ball']['dis'], t['ball']['ang'])
    
    if self.goal_dis == 0:
      self.tStart = t['time']
      self.goal_dis = t['ball']['dis']
    elif t['ball']['dis'] < self.goal_dis:
      self.tStart = t['time']
      self.goal_dis = t['ball']['dis']
    elif t['ball']['dis'] >= self.goal_dis :
      a = self.Calculate(t['time'])
      if a >= 3:
        x, y, yaw = self.Accelerate(x, y, yaw)
        self.tStart = t['time']
        self.goal_dis = t['ball']['dis']

    self.MotionCtrl(x, y, yaw)

  def on_toAttack(self, t, side):
    self.goal_dis = 0 
    x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos):
    self.RobotShoot(power, pos)

  def on_toOrbit(self, t, side):
    x, y, yaw = self.CC.Orbit(t[side]['ang'])
    self.MotionCtrl(x, y, yaw, True)

  def on_toPoint(self, tx, ty, tyaw):
    x, y, yaw, remaining = self.BC.Go2Point(tx, ty, tyaw)
    print("Remaining: ", remaining)
    if remaining >= 40:
      self.MotionCtrl(x, y, yaw)
    return remaining

  def PubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

  def CheckBallHandle(self):
    return self.RobotBallHandle()
  
  def Calculate(self,ntime):
    return ntime - self.tStart
  
  def Accelerate(self,x, y, yaw):
    return x*1.5, y*1.5, yaw

class Strategy(Robot):
  def __init__(self):
    self.game_start = False
    self.game_state = "Kick_Off"
    self.side       = "Yellow"
    self.opp_side   = 'Yellow' if self.side == 'Blue' else 'Blue'

  def RunStatePoint(self, state):
    if state == "Kick_Off" :
      r = self.robot.toPoint(0, 0, 0)
    elif state == "Free_Kick" :
      r = self.robot.toPoint(100, 100, 90)
    elif state == "Free_Ball" :
      r = self.robot.toPoint(100, -100, 180)
    elif state == "Throw_In" :
      r = self.robot.toPoint(-100, -100, 270)
    elif state == "Coner_Kick":< self.goal_dis:
      r = self.robot.toPoint(30< self.goal_dis:
    elif state == "Penalty_Kick" :
      r = self.robot.toPoint(-100, 100, 135)
    elif state == "Run_Specific_Point" :
      r = self.robot.toPoint(self.run_x, self.run_y, 0)
    else:
      print("ummmm")

    if r < 40:
      self.robot.toIdle()

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.run_point  = config['run_point']
    self.side       = config['our_goal']
    self.opp_side   = 'Yellow' if config['our_goal'] == 'Blue' else 'Blue'
    self.run_x      = config['run_x']
    self.run_y      = config['run_y']
    

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    self.run_point = config['run_point']

    return config

  

  def main(self, argv):
    rospy.init_node('core', anonymous=True)
    rate = rospy.Rate(1000)
    dsrv = Server(StrategyConfig, self.Callback)
    dclient = dynamic_reconfigure.client.Client("core", timeout=30, config_callback=None)
    TEST_MODE = True
    if SysCheck(argv) == "Native Mode":
      log("Start Native")
      self.robot = Core(1)
      
    elif SysCheck(argv) == "Simulative Mode":
      log("Start Sim")
      self.robot = Core(1, True)
    
     

    while not rospy.is_shutdown():

      self.robot.PubCurrentState()
      
      
      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()
      #log(position)

      if targets is None or targets['ball']['ang'] == 999 and self.game_start: # Can not find ball when starting
        print("Can not find ball")
        self.robot.toIdle()
      else:
        
        if not self.robot.is_idle and not self.run_point and not self.game_start:
          self.robot.toIdle()
        elif self.robot.is_idle and self.game_start:
          dclient.update_configuration({"run_point": False})
          #self.robot.toChase(targets, self.opp_side, "Straight")
      
          self.robot.toChase(targets, self.opp_side)
        elif self.robot.is_chase:
          #self.robot.toChase(targets, self.opp_side, "Straight")
          self.robot.toChase(targets, self.opp_side)

        if self.robot.is_chase and self.robot.CheckBallHandle():
          self.robot.toAttack(targets, self.opp_side)
        elif self.robot.is_attack:
          self.robot.toAttack(targets, self.opp_side)

        if self.robot.is_attack and not self.robot.CheckBallHandle():
          
          self.robot.toChase(targets, self.opp_side)

        if self.robot.is_attack and abs(targets[self.opp_side]['ang']) < 10:
          self.robot.toShoot(3, 1)

        if self.robot.is_shoot:
          self.robot.toAttack(targets, self.opp_side)

      ## Run point
      if self.run_point and not self.game_start:
        self.RunStatePoint(self.game_state)
        if self.robot.is_point:
          self.RunStatePoint(self.game_state)

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
