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
from methods.block import Block
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import RobotConfig
import dynamic_reconfigure.client

class Core(Robot, StateMachine):

  last_ball_dis = 0
  last_time     = time.time()

  idle    = State('Idle', initial = True)
  block     = State('Block')
  wait      = State('Wait')
  push      = State('Push')
  ret       = State('Ret')
 
  toIdle    = idle.to.itself() | block.to(idle) | wait.to(idle)  | push.to(idle) | ret.to(idle)
  toBlock   = wait.to(block) | block.to.itself() | ret.to(block)  
  toWait    = block.to(wait) | wait.to.itself() 
  toPush    = block.to(push) | push.to.itself() 
  toRet     = idle.to(ret) | push.to(ret) | ret.to.itself() | block.to(ret)

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.run_point  = config['run_point']
    self.our_side   = config['our_side']
    self.opp_side   = 'Blue' if self.our_side == 'Yellow' else 'Yellow'
    self.run_x      = config['run_x']
    self.run_y      = config['run_y']
    self.run_yaw    = config['run_yaw']
    self.maximum_v = config['maximum_v']
    self.ball_speed = config['ball_pwm']
    self.locate = config['locate']

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    return config

  def __init__(self, sim = False):
    super(Core, self).__init__(sim)
    StateMachine.__init__(self)
    self.BC  = Behavior()
    self.BK  = Block()
    self.left_ang = 0
    self.cp_value = 0
    dsrv = DynamicReconfigureServer(RobotConfig, self.Callback)
    

  
  def on_toIdle(self):
    for i in range(0, 10):
        self.MotionCtrl(0,0,0)
    log("To Idle1")
  
  def on_toBlock(self, methods = "Classic"):
    t  = self.GetObjectInfo()
    position = self.GetRobotInfo()
    if methods == "Classic":
      log("to block classic")
      x, y, yaw = self.BK.ClassicBlocking(t['ball']['dis'],\
                                          t['ball']['ang'],\
                                          position['location']['yaw'],\
                                          t['ball']['speed_pwm_x'], t['ball']['speed_pwm_y'],\
                                          self.cp_value)
    elif methods == "Limit":
      log("to block limit")
      x = 0
      y = 0
      yaw = 0
    
    self.MotionCtrl(x, y, yaw)

  def on_toWait(self):

    self.MotionCtrl(0,0,0)

  def on_toRet(self):
    t = self.GetObjectInfo()
    position = self.GetRobotInfo()
    twopoint = self.GetTwopoint()
    if self.locate:
      x, y, yaw, arrived = self.BC.Go2Point(self.run_x, self.run_y, self.run_yaw)
    else :
      x, y, yaw = self.BK.Return(t[self.our_side]['dis'], t[self.our_side]['ang'], position['location']['yaw'], self.cp_value)
      arrived = 0
    
    self.MotionCtrl(x, y, yaw)
    log("Returnig")
    return arrived
  
  def on_toPush(self):
    state = self.game_state
    t = self.GetObjectInfo()
    position = self.GetRobotInfo()
    if state == "Penalty_Kick":
      x, y, yaw = self.BK.GuardPenalting(t['ball']['dis'],\
                                       t['ball']['ang'])
    else:
      x, y, yaw = self.BK.ClassicPushing(t['ball']['dis'],\
                                       t['ball']['ang'],position['location']['yaw'])
    self.MotionCtrl(x, y, yaw)
    log("To Push")
  
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
  def record(self):
    position = self.GetRobotInfo()
    self.cp_value = position['imu_3d']['yaw']     


class Strategy(object):
  def __init__(self, sim=False):
    rospy.init_node('blocker', anonymous=True)
    self.rate = rospy.Rate(200)
    self.robot = Core(sim)
    self.dclient = dynamic_reconfigure.client.Client("blocker", timeout=30, config_callback=None)
    self.main()

  def main(self):
    while not rospy.is_shutdown():      
      self.robot.PubCurrentState()
      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()
      twopoint = self.robot.GetTwopoint()
      state = self.robot.game_state
      our_side = self.robot.our_side
      locate = self.robot.locate
      if targets is None or targets['ball']['ang'] == 999 and self.robot.game_start:
        print("Can not find ball")
        self.robot.toIdle()

      else:
        if not self.robot.is_idle and not self.robot.game_start:
            self.robot.toIdle()

        if self.robot.is_idle:
          if self.robot.game_start:
            if state == "Penalty_Kick":
              self.robot.toPush()
            else:
              self.robot.record()
              self.robot.toRet()

        if self.robot.is_ret:            
          if locate:
            arrived = self.robot.toRet()
            if arrived:
              self.dclient.update_configuration({"locate" : False})
          if targets[our_side]['dis'] <= 90:
            self.robot.toBlock()          
          else:
            self.robot.toRet() 
            
        if self.robot.is_block:
          if targets['ball']['dis'] > 300 and not targets['ball']['dis'] == 999:
            self.robot.toWait()
          else :
            if twopoint[our_side]['left'] > 120 and twopoint[our_side]['left'] > twopoint[our_side]['right'] and targets['ball']['ang'] <= 0:
              self.robot.toBlock('Limit')
            elif twopoint[our_side]['right'] > 120 and twopoint[our_side]['left'] < twopoint[our_side]['right'] and targets['ball']['ang'] >= 0:
              self.robot.toBlock('Limit')
            elif targets['ball']['dis'] <= 45:
                self.robot.toPush()                 
            elif twopoint[our_side]['right'] == 999 or \
               twopoint[our_side]['left'] == 999 and \
               targets['ball']['ang'] == 999:
              self.robot.toRet()
            else:
              self.robot.toBlock()

        if self.robot.is_wait:
          if targets['ball']['dis'] > 300:
            self.robot.toWait()        
          elif targets['ball']['dis'] <= 300:
            self.robot.toBlock()

        if self.robot.is_push:
          if state == "Penalty_Kick":
            if targets[our_side]['dis'] >= 120: 
              self.robot.game_state = "Kick_off"               
              self.robot.toRet()            
            else:                
              self.robot.toPush() 
          elif targets['ball']['dis'] <= 60 and targets[our_side]['dis'] < 120:                 
            self.robot.toPush()            
          else:                
            self.robot.toRet()                     
            
        self.rate.sleep()
        
if __name__ == '__main__':
  try:
    if SysCheck(sys.argv[1:]) == "Native Mode":
      log("Start Native")
      s = Strategy(False)
    elif SysCheck(sys.argv[1:]) == "Simulative Mode":
      log("Start Sim")
      s = Strategy(True)
    # s.main(sys.argv[1:])

  except rospy.ROSInterruptException:
    pass
