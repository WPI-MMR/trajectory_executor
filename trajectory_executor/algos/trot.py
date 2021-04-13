import numpy as np
import abc
import time


class TrotAlgo(abc.ABC):
  def __init__(self, interpolation_steps: int = 5, 
               interpolation_wait: float = 0.005):
    # JOINTS ARE IN DEGREES
    self.joints = {
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
    self._prev_joints = self.joints.copy()
    
    self._inter_steps = interpolation_steps
    self._inter_wait = interpolation_wait
  
  @abc.abstractmethod
  def _send_angles(self):
    pass
  
  def send_angles(self):
    # interpolate the angles
    goal = self.joints.copy()
    for i in range(self._inter_steps):
      for j, v in self.joints.items():
        self.joints[j] = int(v + (goal[j] - v) / int(self._inter_steps) * (i + 1))

      self._send_angles()
      time.sleep(self._inter_wait)
    self._prev_joints = goal

  def FLHR_HFE(self, value):
    self.joints['FL_HFE'] = value
    self.joints['HR_HFE'] = value

  def FLHR_KFE(self, value):
    self.joints['FL_KFE'] = value
    self.joints['HR_KFE'] = value

  def FRHL_HFE(self, value):
    self.joints['FR_HFE'] = value
    self.joints['HL_HFE'] = value

  def FRHL_KFE(self, value):
    self.joints['FR_KFE'] = value
    self.joints['HL_KFE'] = value

  def prime(self):
    self.joints = {
      'FL_HFE': 35,
      'FL_KFE': -80,
      'FL_ANKLE': 0,
      'FR_HFE': 35,
      'FR_KFE': -80,
      'FR_ANKLE': 0,
      'HL_HFE': -35,
      'HL_KFE': 80,
      'HL_ANKLE': 0,
      'HR_HFE': -35,
      'HR_KFE': 80,
      'HR_ANKLE': 0,
    }
    self.send_angles()

  def reset(self):
    self.joints = {
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
    self.send_angles()

  def run(self):
    self.reset()
    print('Priming in 1 second')
    time.sleep(1)
    # input('Press any key to prime the trot...')

    self.prime()
    input('Press any key to start the trot...')
    
    
if __name__ == '__main__':
  import gym
  import gym_solo

  from gym_solo.envs import solo8v2vanilla_realtime

  import numpy as np
  import time

  class TrotSim(TrotAlgo):
    axis = {
      'FL_HFE': 1,
      'FL_KFE': -1,
      'FL_ANKLE': 1,
      'FR_HFE': -1,
      'FR_KFE': 1,
      'FR_ANKLE': 1,
      'HL_HFE': 1,
      'HL_KFE': 1,
      'HL_ANKLE': 1,
      'HR_HFE': -1,
      'HR_KFE': 1,
      'HR_ANKLE': 1,
    }
    
    def __init__(self, interpolation_steps: int = 5, 
                interpolation_wait: float = 0.005):
      super().__init__(interpolation_steps, interpolation_wait)

      self.config = solo8v2vanilla_realtime.RealtimeSolo8VanillaConfig()
      self.config.urdf_path = 'assets/solo8_URDF_v3/solo8_URDF_v3.urdf'

      # Set the robot to quadrupedal standing
      self.config.starting_joint_pos = {
        'FL_HFE': -np.pi / 4,
        'FL_KFE': -np.pi / 2,
        'FL_ANKLE': 0,
        'FR_HFE': np.pi / 4,
        'FR_KFE': np.pi / 2,
        'FR_ANKLE': 0,
        'HL_HFE': -np.pi / 4,
        'HL_KFE': np.pi / 2,
        'HL_ANKLE': np.pi / 2,
        'HR_HFE': np.pi / 4,
        'HR_KFE': np.pi / 2,
        'HR_ANKLE': np.pi / 2
      }

      self.env = gym.make('solo8vanilla-realtime-v0', config=self.config)

    def _send_angles(self):
      rads = {joint: pos * np.pi / 180 for joint, pos in self.joints.items()}
      action = [(rads[j] * self.axis[j]) + self.config.starting_joint_pos[j] 
                for j in self.env.joint_ordering]
      self.env.step(action)

    def close(self):
      self.env.close()


  sim = TrotSim()
  sim.run()
  sim.close()