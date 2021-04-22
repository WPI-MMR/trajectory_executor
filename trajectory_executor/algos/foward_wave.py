from typing import List, Tuple

from matplotlib import pyplot as plt
import matplotlib.animation as animation

import pandas as pd
import numpy as np
import abc
import collections
import time
import json
import socket
import math


SOCKET_ADDR = ('192.168.1.82', 42069)


class ForwardWave(abc.ABC):
  leg_ordering = ('HL', 'FL', 'HR', 'FR')

  def __init__(self, interpolation_steps: int = 5, 
               interpolation_wait: float = 0.005,
               use_socket: bool = False,
               transfer_phase: List[Tuple[float, Tuple[float, float]]] = None,
               T: float = .75, L: float = 0.24):
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

    self.L1 = 160
    self.L2 = 170
    self.quad_height = 233

    # self.L1 = .2
    # self.L2 = .2
    # self.quad_height = .175

    self.use_socket = use_socket
    if self.use_socket:
      try:
        self.socket = socket.socket()
        self.socket.connect(SOCKET_ADDR)
      except Exception as e:
        print('Error connecting to socket: {}'.format(e))
        self.use_socket = False
    
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
      packet = {k: round(v, 3) for k, v in self.joints.items()}
      self.socket.sendall(json.dumps(packet).encode())
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
      if self.use_socket:
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
        duty_len = self.duty_cycle * self.T 

        delta_x = self.transfer_phase[0][1][0] - self.transfer_phase[-1][1][0]
        m = delta_x / duty_len

        positions[leg] = (self.transfer_phase[-1][1][0] + \
          (duty_len - distance_to_start) * m, 0)
        
    return positions
  
  def ikin(self, x, y, is_front_leg):
    """Returns the joint angle of each 2 DOF leg
    """
    alpha, beta = 0, 0  
    alpha  = math.atan2(x, y)
    beta = math.acos((self.L1**2 + x**2 + y**2 - self.L2**2)/(2*self.L1*math.sqrt(x**2 + y**2)))
    if is_front_leg:
        theta1 = alpha - beta
        theta2 = math.acos((x**2 + y**2 - self.L1**2 - self.L2**2)/(2*self.L1*self.L2))
    else:
        theta1 = alpha + beta
        theta2 = -math.acos((x**2 + y**2 - self.L1**2 - self.L2**2)/(2*self.L1*self.L2))

    # Converting theta1 according to the leg 0.
    theta1 += np.pi/2
    return theta1, theta2

  def init_plot(self, ax, leg):
    ax.set_title(leg)
    ax.set_xlim([-330, 330])
    ax.set_ylim([-330, 50])
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    return ax.plot([], [])

  def draw_leg(self, joint1, joint2, line):
    x = [0]
    y = [0]
    
    joint1 = -np.pi/2 + joint1
    
    x1 = self.L1* math.cos(joint1)
    y1 = self.L1*math.sin(joint1)
    
    x.append(x1)
    y.append(y1)
    
    x2 = x1 + self.L2 * math.cos(joint1 + joint2)
    y2 = y1 + self.L2 * math.sin(joint1 + joint2)
    
    x.append(x2)
    y.append(y2)

    line[0].set_data(x, y)

  def animate(self, i, interval):
    t = i * (interval / 1000) % self.T
    pos = self.pos_for_phase(t)

    for leg, (x, y) in pos.items():
      j1, j2 = self.ikin(y * 1000 - self.quad_height, x * 1000, 'F' in leg)
      self.draw_leg(j1, j2, self.lines[leg])
      
    return [l for line in self.lines.values() for l in line]

  def visualize_leg_movements(self):
    frames = 5000
    interval = 10

    self.lines = {}
    fig, axes = plt.subplots(1, 4)
    for i, leg in enumerate(self.leg_ordering):
      self.lines[leg] = self.init_plot(axes[i], leg)

    ani = animation.FuncAnimation(fig, self.animate, frames=frames, 
                                  interval=interval, blit=True, 
                                  fargs=(interval,))
    plt.show()

  def visualize_foot_pos(self):
    space = np.linspace(0, 1, 1000)
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

  def wave(self):
    self.reset()
    input('ready for wave')

    interval = .01

    try:
      i = 0
      while True:
        t = i * interval % self.T
        pos = self.pos_for_phase(t)

        for leg, (x, y) in pos.items():
          # j1, j2 = np.degrees(self.ikin(x, y - self.quad_height / 1000, 'H' in leg))
          j1, j2 = np.degrees(self.ikin(y * 1000 - self.quad_height, x * 1000, 'F' in leg))
          # if 'H' in leg:
          #   j1 += 180
          j1 *= -1
          j2 *= -1

          j1 = j1 if abs(j1) <= 180 else (360 - abs(j1)) * -1 * np.sign(j1)
          j2 = j2 if abs(j2) <= 180 else (360 - abs(j2)) * -1 * np.sign(j2)

          self.joints[f'{leg}_HFE'] = j1
          self.joints[f'{leg}_KFE'] = j2
        
        self.send_angles()
        i += 1
        time.sleep(interval)

    except KeyboardInterrupt as e:
      pass

  def generate_checkpoints(self, freq):
    cum_joints = []

    checkpoints = np.arange(0, self.T, freq)    
    for t in checkpoints:
      print(t)
      pos = self.pos_for_phase(t)

      joints = {}
      for leg, (x, y) in pos.items():
        j1, j2 = np.degrees(self.ikin(y * 1000 - self.quad_height, x * 1000, 'F' in leg))

        j1 *= -1
        j2 *= -1

        j1 = j1 if abs(j1) <= 180 else (360 - abs(j1)) * -1 * np.sign(j1)
        j2 = j2 if abs(j2) <= 180 else (360 - abs(j2)) * -1 * np.sign(j2)

        joints[f'{leg}_HFE'] = j1
        joints[f'{leg}_KFE'] = j2
      cum_joints.append(joints)

    df = pd.DataFrame(cum_joints)
    df.to_csv('joints.csv', index=False)

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
                         (0.0, (-0.12, 0.0)),
                         (0.037500000000000006, (-0.132, 0.0175)),
                         (0.07500000000000001, (-0.08399999999999998, 0.035)),
                         (0.11249999999999999, (0.02399999999999998, 0.035)),
                         (0.15, (0.072, 0.0175)),
                         (0.1875, (0.06, 0.0))
                       ])

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
      self.env = gym.make('solo8vanilla-realtime-v0', config=self.config)

    def _send_angles(self):
      rads = {joint: pos * np.pi / 180 for joint, pos in self.joints.items()}
      action = [(rads[j] * self.axis[j]) + self.config.starting_joint_pos[j] 
                for j in self.env.joint_ordering]
      self.env.step(action)
    # def _send_angles(self):
    #   rads = {joint: pos for joint, pos in self.joints.items()}
    #   action = [(rads[j] * self.axis[j]) + self.config.starting_joint_pos[j] 
    #             for j in self.env.joint_ordering]
    #   self.env.step(action)

    def close(self):
      self.env.close()


  sim = FowardWaveSim()
  """"
  input()
  interval = .11

  sim.joints = {
    'FL_KFE': 0.1255,
    'FR_KFE': 0.1255,
    'FL_HFE': 0.5618,
    'FR_HFE': 0.5618,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': -1.1908,
    'HR_HFE': -1.1908,
    'HL_KFE': 1.7208,
    'HR_KFE': 1.7208,
    'HL_ANKLE': np.pi / 2 - 0.1300,
    'HR_ANKLE': np.pi / 2 - 0.1300,
  }
  sim._send_angles()
  time.sleep(.2)
  input()

  sim.joints = {
    'FL_KFE': 0.1513,
    'FR_KFE': 0.1513,
    'FL_HFE': 0.5899,
    'FR_HFE': 0.5899,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': -1.0536,
    'HR_HFE': -1.0536,
    'HL_KFE': 1.6579,
    'HR_KFE': 1.6579,
    'HL_ANKLE': np.pi / 2 - 0.1412,
    'HR_ANKLE': np.pi / 2 - 0.1412,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.2670,
    'FR_KFE': 0.2670,
    'FL_HFE': 0.7159,
    'FR_HFE': 0.7159,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': -0.4391,
    'HR_HFE': -0.4391,
    'HL_KFE': 1.3762,
    'HR_KFE': 1.3762,
    'HL_ANKLE': np.pi / 2 - 0.1915,
    'HR_ANKLE': np.pi / 2 - 0.1915,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.4297,
    'FR_KFE': 0.4297,
    'FL_HFE': 0.8931,
    'FR_HFE': 0.8931,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': 0.4255,
    'HR_HFE': 0.4255,
    'HL_KFE': 0.9799,
    'HR_KFE': 0.9799,
    'HL_ANKLE': np.pi / 2 - 0.2621,
    'HR_ANKLE': np.pi / 2 - 0.2621,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.5454,
    'FR_KFE': 0.5454,
    'FL_HFE': 1.0191,
    'FR_HFE': 1.0191,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': 1.0400,
    'HR_HFE': 1.0400,
    'HL_KFE': 0.6983,
    'HR_KFE': 0.6983,
    'HL_ANKLE': np.pi / 2 - 0.3124,
    'HR_ANKLE': np.pi / 2 - 0.3124,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.5712,
    'FR_KFE': 0.5712,
    'FL_HFE': 1.0472,
    'FR_HFE': 1.0472,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': 1.1772,
    'HR_HFE': 1.1772,
    'HL_KFE': 0.6354,
    'HR_KFE': 0.6354,
    'HL_ANKLE': np.pi / 2 - 0.3236,
    'HR_ANKLE': np.pi / 2 - 0.3236
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.5454,
    'FR_KFE': 0.5454,
    'FL_HFE': 1.0191,
    'FR_HFE': 1.0191,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': 1.0400,
    'HR_HFE': 1.0400,
    'HL_KFE': 0.6983,
    'HR_KFE': 0.6983,
    'HL_ANKLE': np.pi / 2 - 0.3124,
    'HR_ANKLE': np.pi / 2 - 0.3124,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.4297,
    'FR_KFE': 0.4297,
    'FL_HFE': 0.8931,
    'FR_HFE': 0.8931,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': 0.4255,
    'HR_HFE': 0.4255,
    'HL_KFE': 0.9799,
    'HR_KFE': 0.9799,
    'HL_ANKLE': np.pi / 2 - 0.2621,
    'HR_ANKLE': np.pi / 2 - 0.2621,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.2670,
    'FR_KFE': 0.2670,
    'FL_HFE': 0.7159,
    'FR_HFE': 0.7159,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': -0.4391,
    'HR_HFE': -0.4391,
    'HL_KFE': 1.3762,
    'HR_KFE': 1.3762,
    'HL_ANKLE': np.pi / 2 - 0.1915,
    'HR_ANKLE': np.pi / 2 - 0.1915,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.1513,
    'FR_KFE': 0.1513,
    'FL_HFE': 0.5899,
    'FR_HFE': 0.5899,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': -1.0536,
    'HR_HFE': -1.0536,
    'HL_KFE': 1.6579,
    'HR_KFE': 1.6579,
    'HL_ANKLE': np.pi / 2 - 0.1412,
    'HR_ANKLE': np.pi / 2 - 0.1412,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
    'FL_KFE': 0.1255,
    'FR_KFE': 0.1255,
    'FL_HFE': 0.5618,
    'FR_HFE': 0.5618,
    'FL_ANKLE': 0,
    'FR_ANKLE': 0,
    'HL_HFE': -1.1908,
    'HR_HFE': -1.1908,
    'HL_KFE': 1.7208,
    'HR_KFE': 1.7208,
    'HL_ANKLE': np.pi / 2 - 0.1300,
    'HR_ANKLE': np.pi / 2 - 0.1300,
  }
  sim._send_angles()
  time.sleep(interval)

  sim.joints = {
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
  sim._send_angles()
  time.sleep(interval)
  input()

  """
  # sim.wave()
  # sim.generate_checkpoints(0.0075)
  # sim.visualize_leg_movements()
  # fig, axes = plt.subplots()
  # line = sim.init_plot(axes, 'Leg')
  # sim.draw_leg(np.pi / 4, -np.pi/3, line)
  # plt.show()
  sim.close()