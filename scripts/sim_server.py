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
import time
import json


PORT = 42069
SIZE = 1024


def env_socket(env, socket):
  while True:
    print('Simulation ready, waiting for client...')
    connection, client_addr = socket.accept()
    
    print(f'Connection from {client_addr}')
    data = connection.recv(SIZE)
    print(data)
  

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
socket.bind(('localhost', PORT))
socket.listen()

socket_thread = threading.Thread(target=env_socket, args=(env, socket))

options = (
  '[r] Reset',
  '[x] Exit'
)

while True:
  menu = TerminalMenu(options, title='Solo Sim Interactive Menu')
  choice_idx = menu.show()
  
  if choice_idx == 0:
    env.reset()
  elif choice_idx == 1:
    socket.close()
    sys.exit(0)
  else:
    print('Invalid choice')