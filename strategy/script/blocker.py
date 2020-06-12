#!/usr/bin/env python
import rospy
import sys
import math
import time
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_sys import log, SysCheck, logInOne
from methods.block import Block
from dynamic_reconfigure.server import Server
from strategy.cfg import StrategyConfig
import dynamic_reconfigure.client

class Core(Robot, StateMachine):
  def __init__(self, robot_num, sim = False):
    super(Core, self).__init__(robot_num, sim)
    StateMachine.__init__(self)
    self.BK  = Block()
    self.sim = sim
  last_ball_dis = 0
  last_time     = time.time()
  idle    = State('Idle', initial = True)

  block     = State('Block')
  wait      = State('Wait')
  r_limit   = State('R_limit')
  l_limit   = State('L_limit')
  ret       = State('Ret')
  guard_penalty = State('Guard_Penalty')
  push      = State('Push')

  toIdle    = idle.to.itself() | block.to(idle) | wait.to(idle) | r_limit.to(idle) | l_limit.to(idle) | ret.to(idle)| guard_penalty.to(idle) |push.to(idle)
  toBlock   = idle.to(block) | wait.to(block) | block.to.itself() | r_limit.to(block) | l_limit.to(block) | ret.to(block) 
  toWait    = idle.to(wait) | block.to(wait) | wait.to.itself() | r_limit.to(wait) | l_limit.to(wait) | guard_penalty.to(wait)
  toR_limit = idle.to(r_limit) | block.to(r_limit) | r_limit.to.itself() 
  toL_limit = idle.to(l_limit) | block.to(l_limit) | l_limit.to.itself() 
  toRet     = idle.to(ret) | ret.to.itself() | guard_penalty.to(ret) | block.to(ret) | push.to(ret)
  toGuard_Penalty = idle.to(guard_penalty) | guard_penalty.to.itself() 
  toPush = block.to(push) | push.to.itself() 
  def on_toIdle(self):
    for i in range(0, 10):
        self.MotionCtrl(0,0,0)
    log("To Idle1")
  
  def on_toBlock(self, t,i):
    x, y, yaw = self.BK.ClassicBlocking(t['ball']['dis'],\
                                        t['ball']['ang'],\
                                        i['imu']['ang'],\
                                        t['ball']['speed_pwm_y'])
    self.MotionCtrl(x, y, yaw)
    log("To Block")
    

  def on_toWait(self,i):
    t=self.GetObjectInfo()
    x, y, yaw = self.BK.ClassicWaiting(i['imu']['ang'])
    self.MotionCtrl(x, y, yaw)
    if not t['ball']['ang']==999:
      log("To Wait")
    else:
        log("Warning:Cannot see ball")
  
  def on_toRet(self, t, side,i):
    x, y, yaw = self.BK.ClassicReturning(t['ball']['ang'],\
                                         t[side]['dis'],\
                                         t[side]['ang'],\
                                         i['imu']['ang'] )
    self.MotionCtrl(x, y, yaw)
    log("To Return")
  
  def on_toR_limit(self,i):
    x, y, yaw = self.BK.ClassicWaiting(i['imu']['ang'])
    self.MotionCtrl(x, y, yaw)
    log("To Rlimit")    

  def on_toL_limit(self,i):
    x, y, yaw = self.BK.ClassicWaiting(i['imu']['ang'])
    self.MotionCtrl(x, y, yaw)
    log("To Llimit")
  
  def on_toGuard_Penalty(self, t,i):
    x, y, yaw = self.BK.ClassicPushung(t['ball']['dis'],\
                                       t['ball']['ang'],\
                                       t['ball']['speed_pwm_x'],\
                                       t['ball']['speed_pwm_y'],\
                                       i['imu']['ang'])
    self.MotionCtrl(x, y, yaw)
    log("To Guard Penalty")

  def on_toPush(self,t,i):
    x, y, yaw = self.BK.ClassicPushung(t['ball']['dis'],\
                                       t['ball']['ang'],\
                                       t['ball']['speed_pwm_x'],\
                                       t['ball']['speed_pwm_y'],\
                                       i['imu']['ang'])
    self.MotionCtrl(x, y, yaw)
    log("To Push")

  def PubCurrentState(self):
    self.RobotStatePub(self.current_state.identifier)

  def CheckBallHandle(self):
    return self.RobotBallHandle()

class Strategy(object):
  def __init__(self, num, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(1000)

    self.robot = Core(num, sim)

    dsrv = Server(StrategyConfig, self.Callback)
    self.dclient = dynamic_reconfigure.client.Client("core", timeout=30, config_callback=None)

  def main(self):

    while not rospy.is_shutdown():
      
      self.robot.PubCurrentState()
      targets = self.robot.GetObjectInfo()
      twopoint = self.robot.GetTwopoint()
      imu = self.robot.GetImu()
      if not self.robot.is_idle and not self.game_start:
          self.robot.toIdle()

    ##idle
      if self.robot.is_idle:
        if self.game_start:
          if self.game_state=="Penalty_Kick":
            self.robot.toGuard_Penalty(targets,imu)
          else:
            self.robot.toBlock(targets,imu)
        
    ##block
      if self.robot.is_block:
        #if targets['ball']['dis']>250:
          #go wait
         # self.robot.toWait(imu)
          
        if twopoint[self.side]['left']>120 and twopoint[self.side]['left']>twopoint[self.side]['right'] and targets['ball']['ang']<=0:
          #go r limit
          self.robot.toR_limit(imu)
          
        elif twopoint[self.side]['right']>120 and twopoint[self.side]['left']<twopoint[self.side]['right'] and targets['ball']['ang']>=0:        
          #go l limit
          self.robot.toL_limit(imu)
          
        elif abs(targets['ball']['ang'])<3:
            #go wait
          self.robot.toWait(imu)

        elif twopoint[self.side]['right']==999 or twopoint[self.side]['left']==999:
          #keep blocking
          self.robot.toBlock(targets,imu)
          
        elif targets['ball']['ang']==999:
          #go wait
          self.robot.toWait(imu)
        
       # elif targets['ball']['dis']<70 and abs(targets['ball']['ang'])<6:
          #go push
         # self.robot.toPush(targets,imu)
        
        elif targets[self.side]['dis']>100:
          #go ret
          self.robot.toRet(targets,self.side,imu)
        
        else:
          #keep blocking
          self.robot.toBlock(targets,imu)
          
     
    ##wait
      if self.robot.is_wait:
        #if targets['ball']['dis']>250:
          #keep waiting
         # self.robot.toWait(imu)
          
        if abs(targets['ball']['ang'])>3 and not targets['ball']['ang']==999:
          #go block
          self.robot.toBlock(targets,imu)
        
        else:
          #keep wait
          self.robot.toWait(imu)
          
        
        
      
    ##r_limit
      if self.robot.is_r_limit: 
        if targets['ball']['ang']>0 and not targets['ball']['ang']==999 or not twopoint[self.side]['left']>120:
          #go block
          self.robot.toBlock(targets,imu)
          
        #if targets['ball']['dis']<=80:
          #go push
          #self.robot.toPush(targets)
          
        elif targets['ball']['ang']==999:
          #go wait
          self.robot.toWait(imu)
        else:
          self.robot.toR_limit(imu)
          
     
    ##l_limit
      if self.robot.is_l_limit:
          
        if targets['ball']['ang']<0 or not twopoint[self.side]['right']>120:
          #go block
          self.robot.toBlock(targets,imu)          
          
        #if targets['ball']['dis']<=80:
          #go push
          #self.robot.toPush(targets)
          
        elif targets['ball']['ang']==999:
          #go wait
          self.robot.toWait(imu)
        else:
          self.robot.toL_limit(imu)
          
    ##push
      if self.robot.is_push:
        if targets[self.side]['dis']>100:  
          #go ret
          self.robot.toRet(targets,self.side,imu)
            
        elif targets['ball']['dis']>70 and not targets['ball']['dis']==999:
          #go ret
          self.robot.toRet(targets,self.side,imu)
          
        elif targets['ball']['ang']==999:
          #go wait
          self.robot.toWait(imu)

        else:
          #go push
          self.robot.toPush(targets,imu)
      
    ##ret
      if self.robot.is_ret:
        if targets[self.side]['dis']<87:
          #go block
          self.robot.toBlock(targets,imu)
          
        else:
          #keep ret
          self.robot.toRet(targets,self.side,imu)

    ##guardpenalty
      if self.robot.is_guard_penalty:
        if targets['ball']['dis']<=150:
          if targets[self.side]['dis']<120:
            #keep push
            self.robot.toGuard_Penalty(targets,imu)
            
          else:
            #go ret
            self.robot.toRet(targets,self.side,imu)
            
        elif targets['ball']['dis']>150 and not targets['ball']['dis']==999:
          #go ret
          self.robot.toRet(targets,self.side,imu)
          
        elif targets['ball']['ang']==999:
          #go wait
          self.robot.toWait(imu)
          
    

      self.rate.sleep()
        

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.run_point  = config['run_point']
    self.side       = config['our_goal']
    self.opp_side   = 'Yellow' if config['our_goal'] == 'Blue' else 'Blue'
    self.run_x      = config['run_x']
    self.run_y      = config['run_y']
    self.run_yaw    = config['run_yaw']
    self.strategy_mode = config['strategy_mode']
    self.orb_attack_ang  = config['orb_attack_ang']
    self.atk_shoot_ang  = config['atk_shoot_ang']
   #self.ROTATE_V_ang   = config['ROTATE_V_ang']
    self.remaining_range_v   = config['remaining_range_v']
    self.remaining_range_yaw = config['remaining_range_yaw']

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
