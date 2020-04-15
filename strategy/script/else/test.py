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
  last_goal_dis = 0
  last_time     = time.time()

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')
  shoot  = State('Shoot')
  point  = State('Point')
  movement = State('Movement')

  toIdle   = level1.to(idle) | level2.to(idle)  | level3.to(idle) | level4.to(idle) | idle.to.itself()
  level1  = toIdle.to(level1)
  level2  = level1.to(level2)
  level3  = level2.to(level3)
  level4  = level3.to(level4)

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
    self.shooting_start = config['shooting_start']
    self.Change_Plan = config['Change_Plan']
    self.atk_shoot_dis = config['atk_shoot_dis']
    self.my_role       = config['role']
    self.accelerate = config['Accelerate']
    self.ball_speed = config['ball_pwm']

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    self.SetMyRole(self.my_role)

    return config

  def __init__(self, sim = False):
    super(Core, self).__init__(sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.AC  = Attack()
    self.BC  = Behavior()
    self.left_ang = 0
    self.dest_angle = 0
    dsrv = DynamicReconfigureServer(RobotConfig, self.Callback)