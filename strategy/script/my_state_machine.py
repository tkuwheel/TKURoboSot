from statemachine import StateMachine, State
from methods.chase import Chase
from methods.attack import Attack
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import RobotConfig
from robot.robot import Robot

class MyStateMachine(Robot, StateMachine):

  def __init__(self, sim = False):
    super(MyStateMachine, self).__init__(sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.AC  = Attack()
    dsrv = DynamicReconfigureServer(RobotConfig, self.Callback)

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.our_side   = config['our_side']
    self.opp_side   = 'Blue' if self.our_side == 'Yellow' else 'Yellow'
    self.maximum_v = config['maximum_v']
    self.minimum_v = config['minimum_v']
    self.atk_shoot_ang = config['atk_shoot_ang']
    self.atk_shoot_dis = config['atk_shoot_dis']

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    return config

  idle      = State('Idle', initial = True)
  chase     = State('Chase')
  attack    = State('Attack')
  shoot     = State('Shoot')
  formation = State('Formation')

  toIdle      = chase.to(idle) | attack.to(idle)  | shoot.to(idle) | formation.to(idle) | idle.to.itself()
  toChase     = idle.to(chase) | attack.to(chase) | chase.to.itself()
  toAttack    = attack.to.itself() | shoot.to(attack) | chase.to(attack)
  toShoot     = attack.to(shoot)| idle.to(shoot)
  toFormation = idle.to(formation) | formation.to.itself()

  def on_toIdle(self):
    self.MotionCtrl(0,0,0)

  def on_toChase(self, method = "Classic"):
    t = self.GetObjectInfo()
    side = self.opp_side
    if method == "Classic":
      x, y, yaw = self.CC.ClassicRounding(t[side]['ang'],\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
    self.MotionCtrl(x, y, yaw)

  def on_toAttack(self, method = "Classic"):
    t = self.GetObjectInfo()
    side = self.opp_side
    if method == "Classic":
      x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])

    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos = 1):
    self.RobotShoot(power, pos)

  def on_toFormation(self):
    m_v, m_gvaw = self.GetMasterGlobalVector()
    # self.MotionCtrl(0, 0, 0)
    # Go2Point_cmd_vel + master_cmd_vel_to_global
    # self.RobotShoot(power, pos)

  def CheckBallHandle(self):
    if self.RobotBallHandle():
      ## Back to normal from Accelerator
      self.ChangeVelocityRange(self.minimum_v, self.maximum_v)
      Core.last_ball_dis = 0

    return self.RobotBallHandle()
