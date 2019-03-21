#!/usr/bin/env python3
import rospy
import sys, getopt
from statemachine import StateMachine, State
from robot.robot import Robot
from std_msgs.msg import String
from my_print import log

class SoccerMachine(StateMachine):
  wait    = State('Wait', initial=True)
  forward = State('Forward')
  back    = State('Back')

  enter = wait.to(forward)
  go_forward = back.to(forward)
  go_back    = forward.to(back)

  def on_enter(self):
    log("I'm running~")
  def on_go_forward(self):
    log("I'm running~")
  def on_go_back(self):
    log("I'm going back.")

class Core(Robot):
  sm = SoccerMachine()
  def __init__(self, robot_num, sim = False):
    self.sim = sim
    super(Core, self).__init__(robot_num)
  
  def Brain(self):
    obj = self.GetObjectInfo()
    if obj['ball']['dis'] is None:
      log("NONE")
    else:
      self.RobotCtrl(50, 0, 0)
      log(self.sm.current_state)

def main(argv):
  rospy.init_node('core', anonymous=True)
  rate = rospy.Rate(50)

  try:
    opts, args = getopt.getopt(argv,"hs",["help", "sim"])
  except getopt.GetoptError:
    log("[ERROR] Append argument --sim to start with simulative mode.", True)
    sys.exit(2)

  if len(argv) < 3:
    log("Native Mode", rosout = True)
    robot = Core(1)
  else:
    for opt, arg in opts:
      if opt in ("-h", "--help"):
        log("Append argument --sim to start with simulative mode.")
        sys.exit()
      elif opt in ("-s", "--sim"):
        log("Simulative Mode", True)
        robot = Core(1, True)

  while not rospy.is_shutdown():
    robot.Brain()
    rate.sleep()

if __name__ == '__main__':
  try:
    main(sys.argv[1:])
  except rospy.ROSInterruptException:
    pass