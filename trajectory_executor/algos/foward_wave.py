from typing import List, Tuple

from matplotlib import pyplot as plt

import numpy as np
import abc
import collections
import time
import json
import socket


SOCKET_ADDR = ('192.168.1.82', 42069)


class ForwardWave(abc.ABC):
  leg_ordering = ('HL', 'FL', 'HR', 'FR')

  def __init__(self, interpolation_steps: int = 5, 
               interpolation_wait: float = 0.005,
               use_socket: bool = True,
               transfer_phase: List[Tuple[float, Tuple[float, float]]] = None,
               T: float = 1, L: float = 0.03):
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

    if use_socket:
      try:
        self.socket = socket.socket()
        self.socket.connect(SOCKET_ADDR)
      except Exception as e:
        print('Error connecting to socket: {}'.format(e))
        use_socket = False
    
    self._inter_steps = interpolation_steps
    self._inter_wait = interpolation_wait

    self.T = T
    self.L = L
    self.transfer_phase = transfer_phase

    self.phase_len, _ = transfer_phase[-1]
    self.duty_cycle = (self.T - self.phase_len) / self.T
    self.transfer_intervals = self.generate_transfer_intervals()
  
  @abc.abstractmethod
  def _send_angles(self):
    pass

  def _send_via_socket(self):
    try:
      self.socket.send(json.dumps(self.joints).encode())
    except Exception as e:
      print('Problem sending {} to {}'.format(self.joints, SOCKET_ADDR))
      print(e)
  
  def send_angles(self):
    # interpolate the angles
    goal = self.joints.copy()
    for i in range(self._inter_steps):
      for j, v in self.joints.items():
        self.joints[j] = int(v + (goal[j] - v) / int(self._inter_steps) * (i + 1))

      self._send_angles()
      if self.socket:
        self._send_via_socket()
      time.sleep(self._inter_wait)
    self._prev_joints = goal

  def generate_transfer_intervals(self):
    transfer_intervals = {}
    for i, leg in enumerate(self.leg_ordering):
      start = self.T / 4 * i
      end = start + self.phase_len

      if end > self.T:
        end = self.T - end

      transfer_intervals[leg] = (start, end)
    return transfer_intervals

  def _in_transfer_interval(self, leg: str, t: float):
    start, end = self.transfer_intervals[leg]
    if start < end:
      return start <= t <= end
    else:
      return t < end or t > start

  def pos_for_phase(self, phi):
    positions = {}
    phase = list(self.transfer_phase)
    
    for leg, (start, end) in self.transfer_intervals.items():
      if self._in_transfer_interval(leg, phi):
        rel_phi = phi - start
        
        for rev_i, (timestep, (x, y)) in enumerate(reversed(phase)):
          i = len(phase) - rev_i - 1
          if timestep == rel_phi:
            positions[leg] = (x, y)
            break

          if rel_phi > timestep:
            next_time, (next_x, next_y) = phase[i + 1]
            
            m_x = (next_x - x) / (next_time - timestep)
            m_y = (next_y - y) / (next_time - timestep)

            inter_x = x + (rel_phi - timestep) * m_x
            inter_y = y + (rel_phi - timestep) * m_y

            positions[leg] = (inter_x, inter_y)
            break
      else:
        if phi < start:
          distance_to_start = start - phi
        else:
          distance_to_start = (self.T - phi) + start
        positions[leg] = (-self.L / 2 + (self.T - distance_to_start) * self.L, 0)
        
    return positions

  def visualize_foot_pos(self):
    space = np.linspace(0, 1)
    pos = collections.defaultdict(list)

    for phi in space:
      positions = self.pos_for_phase(phi)

      for leg in self.leg_ordering:
        pos[leg].append(positions[leg])

    fig, axes = plt.subplots(2, 4)
    fig.suptitle('Horizontal & Vertical Displacement over time')
    plt.setp(axes[0, 0], ylabel='x displacement')
    plt.setp(axes[1, 0], ylabel='y displacement')

    for i, leg in enumerate(self.leg_ordering):
      axes[0, i].set_title(leg)
      axes[0, i].plot(space, [x for x,y in pos[leg]])
      axes[1, i].plot(space, [y for x,y in pos[leg]])

    plt.show()
      
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
    input()
    self.prime()


    input('Launch front left')
    self.joints['FL_HFE'] += -10
    self.joints['FL_KFE'] += 10
    
    self.joints['FR_HFE'] += 10
    self.joints['HL_HFE'] += -10
    self.joints['HR_HFE'] += -10
    self.joints['HR_KFE'] += 10
    self.send_angles()

    input('Extrend front left')
    self.joints['FL_HFE'] += -20
    self.joints['FL_KFE'] += 15
    self.send_angles()

    # Make less abrupt
    input('Lean on to front')
    self.joints['FR_HFE'] += 20
    self.joints['FR_HFE'] += -10
    self.joints['HL_HFE'] += -10
    self.joints['HL_KFE'] += 10

    self.joints['HR_HFE'] += 40
    self.joints['HR_KFE'] += -30
    self.joints['FL_HFE'] = 60
    self.joints['FL_KFE'] += -45
    self.send_angles()

    input('HR step')
    self.joints['HR_HFE'] += -45
    self.joints['HR_KFE'] += 25
    self.send_angles()
   
    input('wait')

    
if __name__ == '__main__':
  import gym
  import gym_solo

  from gym_solo.envs import solo8v2vanilla_realtime

  import numpy as np
  import time

  class FowardWaveSim(ForwardWave):
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
    
    def __init__(self, interpolation_steps: int = 5, 
                interpolation_wait: float = 0.005):
      super().__init__(interpolation_steps, interpolation_wait,
                       transfer_phase=[
                          (0.0, (-0.015, 0.0)),
                          (0.05, (-0.0165, 0.0075)),
                          (0.1, (-0.010499999999999999, 0.015)),
                          (0.15, (0.0029999999999999966, 0.015)),
                          (0.2, (0.009, 0.007499999999999997)),
                          (0.25, (0.0075, -8.673617379884035e-19))])

      self.config = solo8v2vanilla_realtime.RealtimeSolo8VanillaConfig()
      self.config.urdf_path = 'assets/solo8_URDF_v4/solo8_URDF_v4.urdf'

      # Set the robot to quadrupedal standing
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
      # self.env = gym.make('solo8vanilla-realtime-v0', config=self.config)

    def _send_angles(self):
      rads = {joint: pos * np.pi / 180 for joint, pos in self.joints.items()}
      action = [(rads[j] * self.axis[j]) + self.config.starting_joint_pos[j] 
                for j in self.env.joint_ordering]
      self.env.step(action)

    def close(self):
      self.env.close()


  sim = FowardWaveSim()
  # sim.run()
  sim.visualize_foot_pos()
  sim.close()