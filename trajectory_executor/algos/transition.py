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

  def __init__(self, filename):
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
      time.sleep(row['t'])

  def send_angles(self, packet):
    action = [(packet[j] * self.axis[j]) + self.config.starting_joint_pos[j] 
              for j in self.env.joint_ordering]
    self.env.step(action)

if __name__ == '__main__':
  import gym
  import gym_solo
  from gym_solo.envs import solo8v2vanilla_realtime

  gait = TransitionSim('./transition_points.csv')
  input()
  gait.transition()
  input()