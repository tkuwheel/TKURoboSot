#!/usr/bin/env python3
import rospy
import sys
import math
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_sys import log, SysCheck, logInOne
from methods.chase import strategy
from methods.attack import strategy as attack

class SoccerMachine(StateMachine):
  wait    = State('Wait', initial=True)
  chase   = State('Chase')
  attack  = State('Attack')
 
  enter = wait.to(chase) | attack.to(chase)
  stop = chase.to(wait) | attack.to(wait)
  assault = chase.to(attack)

  def on_enter_active(self):
    log("I'm running~")

  def on_enter_wait(self):
    log("I'm stopping.")

  def on_enter_attack(self):
    log("I'm attacking.")

class Core(Robot):
  sm = SoccerMachine()
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
        if obj['ball']['dis'] >= 36 :
          log(self.sm.current_state)
          print("Ball: {}\tCyan Goal: {}\tMagenta Goal: {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal']))
          self.sm.enter()
        if obj['ball']['dis'] <= 36 and abs(obj['magenta_goal']['dis'] <= 55) :
          log("NONE")    
          log(self.sm.current_state)
          print("Ball: {}\tCyan Goal: {}\tMagenta Goal: {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal']))
          sys.exit()
      elif self.sm.is_chase :
        if obj['ball']['dis'] <= 50 and abs(obj['ball']['ang']) <= 20:
            self.sm.assault()
        else:
          ro = strategy(self,obj)
          log(self.sm.current_state)
          self.RobotCtrl(ro['v_x'], ro['v_y'], ro['v_yaw'])
          print("Ball: {}\tCyan Goal: {}\tMagenta Goal: {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal']))
          if obj['ball']['dis'] <= 36 and abs(obj['magenta_goal']['dis'] <= 55) :  
            self.sm.stop()
      elif self.sm.is_attack :
        if obj['ball']['dis'] > 50 or abs(obj['ball']['ang']) > 20 :
          self.sm.enter()
        ro = attack(self,obj)
        log(self.sm.current_state)
        self.RobotCtrl(ro['v_x'], ro['v_y'], ro['v_yaw'])
        print("Ball: {}\tCyan Goal: {}\tMagenta Goal: {}".format(obj['ball'], obj['cyan_goal'], obj['magenta_goal']))

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
    rate.sleep()

if __name__ == '__main__':
  try:
    main(sys.argv[1:])
  except rospy.ROSInterruptException:
    pass