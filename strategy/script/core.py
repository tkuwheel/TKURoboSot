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

  def on_toChase(self, t,side):
    o = self.CC.ClassicRounding(t[side]['ang'],\
                                t['ball']['dis'],\
                                t['ball']['ang'])
    self.RobotCtrl(o['v_x'], o['v_y'], o['v_yaw'])


  def on_toIdle(self):
    log("To Idle")

  def on_toAttack(self, t,side):
    o = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
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
          robot.toChase(targets,self.side)
        elif robot.is_chase and targets['ball']['dis'] >= 37:
          # keep chase
          # log("keep{}".format(targets['ball']['dis']))
          robot.toChase(targets,self.side)
        elif robot.is_chase and targets['ball']['ang'] < 37:
          # go attack
          robot.toAttack(targets,self.side)
        elif robot.is_attack and targets['ball']['dis'] > 30:
          # back chase
          robot.toChase(targets,self.side)

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


  '''
  def on_enter_cross(self):
    log("I'm 轉ing.")

class Core(Robot):
  sm = SoccerMachine()
  bh = 0
  def __init__(self, robot_num, sim = False):
    self.sim = sim
    
    super(Core, self).__init__(robot_num, sim)
  def Brain(self):
#     gains = rospy.get_param("/game_state_server")
#     print("{}, {}, {}".format(gains['game_start'], gains['game_state'], gains['side']))
    obj = self.GetObjectInfo()
    
    if obj['ball']['dis'] == 0:
      log("NONE")  
    else:
      if self.sm.is_wait :
          
        if obj['ball']['dis'] <= 36 and abs(obj['magenta_goal']['dis'] <= 55) :
          log("NONE")    
          log(self.sm.current_state)
          print("Ball:         {}\nCyan Goal:    {}\nMagenta Goal: {}\nVelicity:     {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal'],obj['velicity']))
          sys.exit()
        else :
          log(self.sm.current_state)
          print("Ball:         {}\nCyan Goal:    {}\nMagenta Goal: {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal']))
          self.sm.enter()



      elif self.sm.is_chase :
        if obj['ball']['dis'] <= 50 and abs(obj['ball']['ang']) <= 20:
          self.sm.assault()
        
        elif self.bh == 0 and obj['ball']['dis'] <= 50:
          self.sm.circle()
        else:
          ro = strategy(self,obj)
          log(self.sm.current_state)
          if self.bh == 0:
            ro['v_yaw'] = obj['ball']['ang']
          self.RobotCtrl(ro['v_x'], ro['v_y'], ro['v_yaw'])
          print("Ball: {}\tCyan Goal: {}\tMagenta Goal: {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal']))
          if obj['ball']['dis'] <= 36 and abs(obj['magenta_goal']['dis'] <= 55) :  
            self.sm.stop()






      elif self.sm.is_attack :
        if obj['ball']['dis'] > 50 or abs(obj['ball']['ang']) > 20  :
          self.sm.enter()
        else:
          ro = attack(self,obj)
          log(self.sm.current_state)
          self.RobotCtrl(ro['v_x'], ro['v_y'], ro['v_yaw'])
          print("Ball:         {}\nCyan Goal:    {}\nMagenta Goal: {}\nVelicity:     {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal'],obj['velocity']))
          



      elif self.sm.is_cross :
        if obj['ball']['dis'] <= 40:
          ro = rotate(self,obj)
          log(self.sm.current_state)
          obj['velicity'] = math.hypot(ro['v_x'], ro['v_y'])
          self.RobotCtrl(ro['v_x'], ro['v_y'], ro['v_yaw'])
          print("Ball:         {}\nCyan Goal:    {}\nMagenta Goal: {}\nVelicity:     {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal'],obj['velicity']))
          if  obj['magenta_goal']['ang'] <= 15  and obj['magenta_goal']['ang'] > 0 :
            self.bh = 1
            self.sm.stop()

        else:
          ro = rotate(self,obj)
          log(self.sm.current_state)
          obj['velicity'] = math.hypot(ro['v_x'], ro['v_y'])
          self.RobotCtrl(ro['v_x'], ro['v_y'], ro['v_yaw'])
          print("Ball:         {}\nCyan Goal:    {}\nMagenta Goal: {}\nVelicity:     {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal'],obj['velocity']))

        
        

          
            
       

def main(argv):
  rospy.init_node('core', anonymous=True)
  rate = rospy.Rate(50)

  if SysCheck(argv) == "Native Mode":
    log("Start Native")
    robot = Core(1)
  elif SysCheck(argv) == "Simulative Mode":
    log("Start Sim")
    robot = Core(1, True)

  while not rospy.is_shutdown():
    robot.Brain()
    
    '''