import pandas as pd
import numpy as np
import abc
import time


class TransitionSim(abc.ABC):
  axis = {
    'FL_HFE': 1,
    'FL_KFE': 1,
    'FL_ANKLE': 1,
    'FR_HFE': -1,
    'FR_KFE': -1,
    'FR_ANKLE': 1,
    'HL_HFE': 1,
    'HL_KFE': 1,
    'HL_ANKLE': 1,
    'HR_HFE': -1,
    'HR_KFE': -1,
    'HR_ANKLE': -1,
  }

  def __init__(self, filename, front_mult=1.2, back_mult=1.2):
    self.trajectory = pd.read_csv(
      filename, names=['ankle', 'knee', 'hip', 'shoulder', 'elbow', 't'])
    
    self.config = solo8v2vanilla_realtime.RealtimeSolo8VanillaConfig()
    self.config.urdf_path = 'assets/solo8_URDF_v4/solo8_URDF_v4.urdf'
    self.config.starting_joint_pos = {
      'FL_HFE': 0,
      'FL_KFE': 0,
      'FL_ANKLE': 0,
      'FR_HFE': 0,
      'FR_KFE': 0,
      'FR_ANKLE': 0,
      'HL_HFE': 0,
      'HL_KFE': 0,
      'HL_ANKLE': 0,
      'HR_HFE': 0,
      'HR_KFE': 0,
      'HR_ANKLE': 0,
    }
    self.env = gym.make('solo8vanilla-realtime-v0', config=self.config)
    self.env.obs_factory.register_observation(CompliantObs(self.env.robot))

    self.front_mult = front_mult
    self.back_mult = back_mult
    self.update_mass()

  def update_mass(self):
    for leg, i in [('FL_HFE', 6),('FL_KFE', 7),('FR_HFE', 8),('FR_KFE', 9)]:
      current_mass = self.env.client.getDynamicsInfo(self.env.robot, i)[0]

      if 'F' in leg:
        self.env.client.changeDynamics(self.env.robot, i, mass=current_mass * self.front_mult)
      else:
        self.env.client.changeDynamics(self.env.robot, i, mass=current_mass * self.back_mult)

  def transition(self):
    for _, row in self.trajectory.iterrows():
      joint_packet = {
        'FL_KFE': row['elbow'],
        'FR_KFE': row['elbow'],
        'FL_HFE': row['shoulder'],
        'FR_HFE': row['shoulder'],
        'HL_HFE': -row['hip'],
        'HR_HFE': -row['hip'],
        'HL_KFE': -row['knee'],
        'HR_KFE': -row['knee'],
        'HL_ANKLE': 1.182 - row['ankle'], 
        'HR_ANKLE': 1.182 - row['ankle'],
      }
      self.send_angles(joint_packet)
      self.drop_altitude(row['t'])
      time.sleep(row['t'])

  def send_angles(self, packet):
    action = [(packet[j] * self.axis[j]) + self.config.starting_joint_pos[j] 
              for j in self.env.joint_ordering]
    self.env.step(action)

  def drop_altitude(self, interval):
    base_pos, base_orien = self.env.client.getBasePositionAndOrientation(self.env.robot)
    com_pos = self.env.client.getDynamicsInfo(self.env.robot, -1)[3]
    com_orien = self.env.client.getDynamicsInfo(self.env.robot, -1)[4]

    global_pos, _ = self.env.client.multiplyTransforms(base_pos, base_orien, com_pos, com_orien)
    self.env.client.addUserDebugLine(global_pos, [global_pos[0], global_pos[1], 0], 
                                     lineColorRGB=[255, 0, 0], 
                                     lineWidth=5, 
                                     lifeTime=interval)

  def debug(self):
    pass
      
  
  def compute_com(self, interval):
    base_pos, base_orien = self.env.client.getBasePositionAndOrientation(self.env.robot)

    com_pos = self.env.client.getDynamicsInfo(self.env.robot, -1)[3]
    com_orien = self.env.client.getDynamicsInfo(self.env.robot, -1)[4]

    global_pos, _ = self.env.client.multiplyTransforms(base_pos, base_orien, com_pos, com_orien)
    self.env.client.addUserDebugLine(global_pos, [global_pos[0], global_pos[1], 0], 
                                      lineColorRGB=[255, 0, 0], 
                                      lineWidth=5, 
                                      lifeTime=interval)


if __name__ == '__main__':
  import gym
  import gym_solo
  from gym_solo.envs import solo8v2vanilla_realtime
  from gym_solo.testing import CompliantObs

  bounds = (1.5, 2)
  mult = (bounds[0] + bounds[1])/2
  gait = TransitionSim('./transition_points.csv', front_mult=mult, back_mult=mult)
  while True:
  # gait.debug()
    print()
    print(f'Imbalance: {mult}')

    gait.transition()
    time.sleep(.1)
    (x, y, z), _ = gait.env.client.getBasePositionAndOrientation(gait.env.robot)
    print(x, y, z)

    if x > 0:
      mult = (bounds[1] + mult) / 2
    else:
      mult = (bounds[0] + mult) / 2

    gait.front_mult = mult
    gait.back_mult = mult
    gait.update_mass()
    gait.env.reset()
