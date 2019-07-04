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
    self.shoot = 0


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
    elif method == "Team":
      x, y, yaw = self.CC.TeamWork(t['ball']['dis'], t['ball']['ang']) 
    # self.Accelerate(1,t)
    self.MotionCtrl(x, y, yaw)

  def on_toAttack(self, t, side, run, method = "block"):
    robot_info = self.GetRobotInfo()
    
    if method == "Classic":
      x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
      self.MotionCtrl(x, y, yaw)
      if abs(t[side]['ang']) < 5 :
        self.shoot = 1
    elif method == "block":
     
      x, y, yaw, self.shoot= self.AC.block_attack(t, side, run)
      self.MotionCtrl(x, y, yaw)

      
      

  def on_toShoot(self, power, pos):
    self.RobotShoot(power, pos)
    self.shoot = 0

  def on_toOrbit(self, t, side):
    x, y, yaw = self.CC.Orbit(t[side]['ang'])
    self.MotionCtrl(x, y, yaw, True)

  def on_toPoint(self, tx, ty, tyaw):
    x, y, yaw, remaining = self.BC.Go2Point(tx, ty, tyaw)
    print("got it: ", remaining)
    self.MotionCtrl(x, y, yaw)
    return remaining

  def PubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

  def CheckBallHandle(self):
    return self.RobotBallHandle()
  
  def Calculate(self,ntime):
    return ntime - self.tStart
  
  def Accelerate(self, do, t, maximum_v = 100):
    if do :
      if self.goal_dis == 0:
        print('goal into')
        self.tStart = t['time']
        self.goal_dis = t['ball']['dis']
      elif t['ball']['dis'] < self.goal_dis:
        self.tStart = t['time']
        self.goal_dis = t['ball']['dis']
      elif t['ball']['dis'] >= self.goal_dis :
        a = self.Calculate(t['time'])       
        self.goal_dis = t['ball']['dis']
        print('accelerating')
        self.ChangeVelocityRange(0,maximum_v)
    else :
      self.ChangeVelocityRange(0,maximum_v)
      print('back to normal')
      

class Strategy(object):
  def __init__(self, num, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(1000)
    self.run = {'point' : {'x' : 0, 'y' : 0, 'yaw' : 0}}
    self.side = {'opponent' : '0', 'teamate' : '0', 'pos' : '0'}
    self.robot = Core(num, sim)
    dsrv = Server(StrategyConfig, self.Callback)
    self.dclient = dynamic_reconfigure.client.Client("core", timeout=30, config_callback=None)

  def RunStatePoint(self, state):
    if state == "Kick_Off" and self.side == "Yellow" :
      c = self.robot.toPoint(-60, 0, 0)
    elif state == "Kick_Off" and self.side == "Blue" :
      c = self.robot.toPoint(60, 0, 180)
    elif state == "Free_Kick" :
      c = self.robot.toPoint(100, 100, 90)
    elif state == "Free_Ball" :
      c = self.robot.toPoint(100, -100, 180)
    elif state == "Throw_In" :
      c = self.robot.toPoint(-100, -100, 270)
    elif state == "Coner_Kick":
      c = self.robot.toPoint(300, 200, 45)
    elif state == "Penalty_Kick" :
      c = self.robot.toPoint(-100, 100, 135)
    elif state == "Run_Specific_Point" :
      c = self.robot.toPoint(self.run_x, self.run_y, self.run_yaw)
    else:
      print("ummmm")

    if c:
      self.robot.toIdle()
      self.dclient.update_configuration({"run_point": False})

  def Chase(self, t):
    if self.strategy_mode == "Defense":
      return self.robot.toChase(t, self.side['opponet'], "Classic")
    elif self.strategy_mode == "Attack":
      return self.robot.toChase(t, self.side['opponet'], "Straight")
      #return self.robot.toChase(t, self.side['opponet'], "Teamwork")

      

  def Attack(self, t):
    if self.strategy_mode == "Defense":
      return self.robot.toAttack(t, self.side['opponet'], "Classic")
    elif self.strategy_mode == "zone":
      return self.robot.toAttack(t, self.side['opponet'], "Classic")

  

  def main(self):
    while not rospy.is_shutdown():

      self.robot.PubCurrentState()

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()

      if targets is None or targets['ball']['ang'] == 999 and self.game_start: # Can not find ball when starting
        print("Can not find ball")
        self.robot.toIdle()
      else:
        
        if not self.robot.is_idle and not self.run_point and not self.game_start:
          self.robot.toIdle()
        if self.robot.is_idle:
          
          if self.game_start:
            self.Chase(targets)
          elif self.run_point:
            self.RunStatePoint(self.game_state)

        if self.robot.is_chase:
          if self.robot.CheckBallHandle():
            self.robot.goal_dis = 0
            #self.robot.Accelerate(0,targets,self.maximum_v) 
            if self.strategy_mode == "Attack":
              self.robot.toOrbit(targets, self.side['opponet'])
            elif self.strategy_mode == "Defense":
              self.robot.toAttack(targets, self.side['opponet'], self.run)
          else:
            self.Chase(targets)

        if self.robot.is_orbit:
          if abs(targets[self.side['opponet']]['ang']) < 10:
            self.robot.toAttack(targets, self.side['opponet'], self.run)
          elif not self.robot.CheckBallHandle():
            self.Chase(targets)
          else:
            self.robot.toOrbit(targets, self.side['opponet'])

        if self.robot.is_attack:
          if not self.robot.CheckBallHandle():
            self.Chase(targets)
          elif  self.robot.shoot:
            self.robot.toShoot(3, 1)
          else:
            self.robot.toAttack(targets, self.side['opponet'], self.run)

            

        if self.robot.is_shoot:
          self.robot.toAttack(targets, self.side['opponet'], self.run)

      ## Run point
      if self.robot.is_point:
        self.RunStatePoint(self.game_state)

      if rospy.is_shutdown():
        log('shutdown')
        break

      self.rate.sleep()

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.run_point  = config['run_point']
    self.side['pos']            = 'attack' 
    self.side['teamate']        = config['our_goal']
    self.side['opponet']        = 'Yellow' if config['our_goal'] == 'Blue' else 'Blue'
    self.run['point']['x']      = config['run_x']
    self.run['point']['y']      = config['run_y']
    self.run['point']['yaw']    = config['run_yaw']
    self.strategy_mode = config['strategy_mode']
    self.maximum_v = config['maximum_v']
    

    self.robot.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.robot.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.robot.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    self.run_point = config['run_point']

    return config

if __name__ == '__main__':
  try:
    if SysCheck(sys.argv[1:]) == "Native Mode":
      log("Start Native")
      s = Strategy(1, False)
    elif SysCheck(sys.argv[1:]) == "Simulative Mode":
      log("Start Sim")
      s = Strategy(1, True)
    # s.main(sys.argv[1:])
    s.main()
  except rospy.ROSInterruptException:
    pass
