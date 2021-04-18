import gym
import gym_solo

from gym_solo.envs import solo8v2vanilla_realtime
from gym_solo.core import obs
from gym_solo import testing

from  simple_term_menu import TerminalMenu
import numpy as np
import socket
import sys
import threading
import json


PORT = 42069
SIZE = 1024


def env_socket(env, socket):
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

  while True:
    print('Simulation ready, waiting for client...')
    connection, client_addr = socket.accept()
    
    print(f'Connection from {client_addr}')
    while connection:
      try:
        data = json.loads(connection.recv(SIZE).decode())
        rads = {joint: pos * np.pi / 180 for joint, pos in data.items()} 
        action = [rads[j] * axis[j] for j in env.joint_ordering]
        print(action)
        env.step(action)
      except:
        break
  

config = solo8v2vanilla_realtime.RealtimeSolo8VanillaConfig()
config.urdf_path = 'assets/solo8_URDF_v4/solo8_URDF_v4.urdf'

# Set the robot to quadrupedal standing
config.starting_joint_pos = {
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
env = gym.make('solo8vanilla-realtime-v0', config=config)
env.obs_factory.register_observation(testing.CompliantObs(env.robot))


socket = socket.socket()
socket.bind(('', PORT))
socket.listen()

socket_thread = threading.Thread(target=env_socket, args=(env, socket))
socket_thread.start()


try:
  print('Simulation active. Prese enter to reset and ^C to exit.')
  while True:
    input()
    env.reset()
except KeyboardInterrupt:
  socket.close()
  sys.exit(0)