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

  toIdle   = chase.to(idle) | attack.to(idle)  | movement.to(idle) | point.to(idle) | shoot.to(idle) | idle.to.itself()
  toChase  = idle.to(chase) | attack.to(chase) | chase.to.itself() | movement.to(chase) | point.to(chase)
  toAttack = attack.to.itself() | shoot.to(attack) | movement.to(attack)| chase.to(attack)| point.to(attack)
  toShoot  = attack.to(shoot)| idle.to(shoot)|movement.to(shoot)
  toMovement = chase.to(movement) | movement.to.itself()| idle.to(movement) | point.to(movement) 
  toPoint  = point.to.itself() | idle.to(point) | movement.to(point) | chase.to(point) 

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.game_state = config['game_state']
    self.chase_straight = config['chase_straight']
    self.run_point  = config['run_point']
    self.our_side   = config['our_side']
    self.opp_side   = 'Blue' if self.our_side == 'Yellow' else 'Yellow'
    self.run_x      = config['run_x']
    self.run_y      = config['run_y']
    self.run_yaw    = config['run_yaw']
    self.strategy_mode = config['strategy_mode']
    self.attack_mode = config['attack_mode']
    self.maximum_v = config['maximum_v']
    self.orb_attack_ang = config['orb_attack_ang']
    self.atk_shoot_ang = config['atk_shoot_ang']
    self.my_role       = config['role']
    self.atk_shoot_dis = config['atk_shoot_dis']
    self.accelerate = config['Accelerate']
    self.ball_speed = config['ball_pwm']

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    return config

  def __init__(self, sim = False):
    super(Core, self).__init__(sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.AC  = Attack()
    self.BC  = Behavior()
    self.left_ang = 0

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

    elif method == "Defense":
      x, y, yaw = self.AC.Defense(t['ball']['dis'], t['ball']['ang'])
    if self.accelerate:
      print("accccccc")
      self.Accelerator(80)
    if self.ball_speed:
      print("baaaaaaaaa")
      x = x + t['ball']['speed_pwm_x']
      y = y + t['ball']['speed_pwm_y']
      
      
    self.MotionCtrl(x, y, yaw)

  def on_toAttack(self, method = "Classic"):
    
    t = self.GetObjectInfo()
    side = self.opp_side
    l = self.GetObstacleInfo()
    if method == "Classic":
      x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
    elif method == "Cut":
      x, y, yaw = self.AC.Cut(t[side]['dis'], t[side]['ang'],self.run_yaw)
    elif method == "Post_up":
      if t[side]['dis'] < 50 :
        t[side]['dis'] = 50
      x, y, yaw = self.AC.Post_up(t[side]['dis'],\
                                       t[side]['ang'],\
                                       l['ranges'],\
                                       l['angle']['increment'])
    elif method == "Orbit":
      x, y, yaw, arrived = self.BC.Orbit(t[side]['ang'])
      self.MotionCtrl(x, y, yaw, True)
       
    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos = 1):
    self.RobotShoot(power, pos)

  def on_toMovement(self, method):
    t = self.GetObjectInfo() 
    position = self.GetRobotInfo()
    side = self.opp_side
    ourside = self.our_side
    l = self.GetObstacleInfo()
    log('move')
    if method == "Orbit":
      x, y, yaw, arrived = self.BC.Orbit(t[side]['ang'])
      self.MotionCtrl(x, y, yaw, True)

    elif method == "Relative_ball":
      x, y, yaw = self.BC.relative_ball(t[ourside]['dis'],\
                                             t[ourside]['ang'],\
                                             t['ball']['dis'],\
                                             t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)
   
    elif method == "Relative_goal":
      x, y, yaw = self.BC.relative_goal(t[ourside]['dis'],\
                                             t[ourside]['ang'],\
                                             t['ball']['dis'],\
                                             t['ball']['ang'])
      self.MotionCtrl(x, y, yaw)

    elif method == "Penalty_Kick":
      x, y, yaw = self.BC.PenaltyTurning(side, self.run_yaw)
      self.left_ang = abs(yaw)
      self.MotionCtrl(x, y, yaw )
      
    elif method == "At_Post_up":
      x, y, yaw = self.BC.Post_up(t[side]['dis'],\
                                       t[side]['ang'],\
                                       l['ranges'],\
                                       l['angle']['increment'])
      
      self.MotionCtrl(x, y, yaw)

  def on_toPoint(self):
    t = self.GetObjectInfo()
    our_side = self.our_side
    opp_side = self.opp_side
    #if self.game_state == "Kick_Off" and self.our_side == "Yellow" :
    #  x, y, yaw, arrived = self.BC.Go2Point(-60, 0, 0)
    #elif self.game_state == "Kick_Off" and self.our_side == "Blue" :
    #  x, y, yaw, arrived = self.BC.Go2Point(60, 0, 180)
    #elif self.game_state == "Free_Kick" :
    #  x, y, yaw, arrived = self.BC.Go2Point(100, 100, 90)
    #elif self.game_state == "Free_Ball" :
    #  x, y, yaw, arrived = self.BC.Go2Point(-100, -100, 180)
    #elif self.game_state == "Throw_In" :
    #  x, y, yaw, arrived = self.BC.Go2Point(-100, -100, 270)
    #elif self.game_state == "Coner_Kick":
    #  x, y, yaw, arrived = self.BC.Go2Point(300, 200, 45)
    #elif self.game_state == "Penalty_Kick" :
    #  x, y, yaw, arrived = self.BC.Go2Point(-100, 100, 135)
    #elif self.game_state == "Run_Specific_Point" :
    if self.run_yaw == 0:
      yaw = t[our_side]['ang']
    elif self.run_yaw == 180:
      yaw = t[opp_side]['ang']
    elif self.run_yaw == -180:
      yaw = t['ball']['ang']
    else :
      yaw = self.run_yaw
    x, y, yaw, arrived = self.BC.Go2Point(self.run_x, self.run_y, yaw)
    #else:
      #log("Unknown Game State")

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

  def __init__(self, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(200)
    self.robot = Core(sim)
    self.dclient = dynamic_reconfigure.client.Client("core", timeout=30, config_callback=None)
    self.main()

  def RunStatePoint(self):
    print("run point")
    if self.robot.run_point == "ball_hand":
      if self.robot.toPoint():
        self.dclient.update_configuration({"run_point": "none"})
        self.ToMovement()
    elif self.robot.run_point == "empty_hand":
      if self.robot.toPoint():
        self.dclient.update_configuration({"run_point": "none"})
        self.ToChase()




  def ToChase(self):
    mode = self.robot.attack_mode
    
    if mode == "Defense":
      self.ToMovement()

    else:
      if not self.robot.chase_straight :
        self.robot.toChase("Classic")
      else:
        self.robot.toChase("Straight")

  def ToAttack(self):
    mode = self.robot.attack_mode
    if mode == "Attack" :
      self.robot.toAttack("Classic")
    elif mode == "Cut":
      self.robot.toAttack("Cut")
    elif mode == "Post_up":
      self.robot.toAttack("Post_up")
    elif mode == "Orbit":
      self.robot.toAttack("Orbit")

  def ToMovement(self):
    mode = self.robot.strategy_mode
    state = self.robot.game_state
    point = self.robot.run_point
    log(point)
    if point == "ball_hand":
      print("ppppppppppp")
      self.RunStatePoint()
    elif state == "Penalty_Kick":
      self.robot.toMovement("Penalty_Kick")
    elif mode == "At_Post_up":
      log("movement")
      self.robot.toMovement("At_Post_up")
    elif mode == "At_Orbit":
      self.robot.toMovement("Orbit")
    elif mode == "Defense_ball":
      self.robot.toMovement("Relative_ball")
    elif mode == "Defense_goal":
      self.robot.toMovement("Relative_goal")
    elif mode == "Fast_break":
      self.ToAttack()
  
  def main(self):
    while not rospy.is_shutdown():
      self.robot.PubCurrentState()
      self.robot.Supervisor()

      print("My Namespace: {}, My Role: {}".format(rospy.get_namespace(), self.robot.MyRole(rospy.get_namespace())))

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()
      mode = self.robot.strategy_mode
      state = self.robot.game_state
      laser = self.robot.GetObstacleInfo()
      point = self.robot.run_point

      # Can not find ball when starting
      if targets is None or targets['ball']['ang'] == 999 and self.robot.game_start:
        print("Can not find ball")
        self.robot.toIdle()
      else:
        if not self.robot.is_idle and not self.robot.game_start:
          self.robot.toIdle()

        if self.robot.is_idle:          
          if self.robot.game_start:
            if state == "Corner_Kick" or state == "Free_Kick" or state == "Throw_In":
              self.robot.toShoot(80)

            elif state == "Penalty_Kick":
              self.ToMovement()
            elif self.robot.run_point == "empty_hand":
              log(123)
              self.RunStatePoint()
            else :
              print('idle to chase')
              self.ToChase()
              
        if self.robot.is_chase:
          if self.robot.CheckBallHandle():
            print('chase to move')
            # self.robot.goal_dis = 0
            # self.robot.Accelerate(0,targets,self.maximum_v) 
            self.ToMovement()
          else:
            self.ToChase()

        if self.robot.is_movement:          
          if state == "Penalty_Kick":
            if self.robot.left_ang <= self.robot.atk_shoot_ang:
              print("stop") 
              self.robot.game_state = "Kick_Off"
              self.robot.toShoot(100)
            else:
              self.ToMovement()
                    
          elif mode == 'At_Orbit':
            if abs(targets[self.robot.opp_side]['ang']) < self.robot.orb_attack_ang:
              self.ToAttack()
            elif not self.robot.CheckBallHandle():
              self.ToChase()
            else:
              self.ToMovement()

          elif mode == 'At_Post_up':
            if targets[self.robot.opp_side]['dis'] <= self.robot.atk_shoot_dis:
              self.ToAttack()
            elif not self.robot.CheckBallHandle():
                self.ToChase()
            else:
              self.ToMovement()              

          elif mode == "Defense_ball" or mode == "Defense_goal":  
            if self.robot.CheckBallHandle():
              self.dclient.update_configuration({"strategy_mode": "Fast_break"})
              self.dclient.update_configuration({"attack_mode": "Attack"})

              self.ToChase()
            else : 
              self.ToMovement()

          elif mode == "Fast_break":
            
            self.ToAttack()

        if self.robot.is_attack:
          if not self.robot.CheckBallHandle():
            self.ToChase()
          elif  abs(targets[self.robot.opp_side]['ang']) < self.robot.atk_shoot_ang and \
                abs(targets[self.robot.opp_side]['dis']) < self.robot.atk_shoot_dis:
            self.robot.toShoot(100)
          else:
            self.ToAttack()

        if self.robot.is_shoot:
          self.ToAttack()

      ## Run point
        if self.robot.is_point:
          if point == "ball_hand" :
            if self.robot.CheckBallHandle():
              self.RunStatePoint()
            else:
              self.ToChase()
          else:
            self.RunStatePoint()


        if rospy.is_shutdown():
          log('shutdown')
          break

        self.rate.sleep()

if __name__ == '__main__':
  try:
    if SysCheck(sys.argv[1:]) == "Native Mode":
      log("Start Native")
      s = Strategy(False)
    elif SysCheck(sys.argv[1:]) == "Simulative Mode":
      log("Start Sim")  
      s = Strategy(True)
  except rospy.ROSInterruptException:
    pass
