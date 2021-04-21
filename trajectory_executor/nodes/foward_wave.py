import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest
from trajectory_executor.algos.foward_wave import ForwardWave

from simple_term_menu import TerminalMenu
from pathlib import Path
import pandas as pd
import time


def _set_angle_FOR(angle):
  if angle < 0:
    angle = 360 + angle
  return int(angle)


class FowardWaveNode(Node, ForwardWave):
  def __init__(self, args, interpolation_steps: int = 5, 
              interpolation_wait: float = 0.005):
    rclpy.init(args=args)

    Node.__init__(self, 'trot')
    ForwardWave.__init__(self, interpolation_steps, interpolation_wait,
                         transfer_phase=[
                           (0.0, (-0.05, 0.0)),
                           (0.025, (-0.05500000000000001, 0.0075)),
                           (0.05, (-0.034999999999999996, 0.015)),
                           (0.075, (0.010000000000000009, 0.015)),
                           (0.1, (0.03000000000000002, 0.007499999999999997)),
                           (0.125, (0.025000000000000022, -8.673617379884035e-19)),
                         ])

    # Create publisher
    self.joint_publisher = self.create_publisher(
      JointAngles, 'joint_angles', 10)

    # Create to ensure that the connection is up with com_serial. Not actually 
    # used.
    self.sensors_client = self.create_client(
      SensorDataRequest, 'sensor_data_request')

    while not self.sensors_client.wait_for_service(timeout_sec=1.):
      self.get_logger().warning('Serial service inactive, waiting...')

  def _send_angles(self):
    joint_msg = JointAngles()
    
    joint_msg.left_hip = _set_angle_FOR(self.joints['HL_HFE'])
    joint_msg.left_knee = _set_angle_FOR(self.joints['HL_KFE'])
    joint_msg.right_hip = _set_angle_FOR(self.joints['HR_HFE'])
    joint_msg.right_knee = _set_angle_FOR(self.joints['HR_KFE'])
    joint_msg.left_shoulder = _set_angle_FOR(self.joints['FL_HFE'])
    joint_msg.left_elbow = _set_angle_FOR(self.joints['FL_KFE'])
    joint_msg.right_shoulder = _set_angle_FOR(self.joints['FR_HFE'])
    joint_msg.right_elbow = _set_angle_FOR(self.joints['FR_KFE'])

    self.get_logger().debug('Sending the following robot configuration: {}'.format(self.joints))
    self.joint_publisher.publish(joint_msg)

  def close(self):
    self.destroy_node()
    rclpy.shutdown()


def wave(args=None):
  gait = FowardWaveNode(args, interpolation_steps=5, interpolation_wait=0.005)
  gait.joints = {
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
  interval = 0.01

  checkpoints = pd.read_csv('~/joints.csv')
  try:
    while True:
      for _, row in checkpoints.iterrows():
        for k, v in row.items():
          gait.joints[k] = v
        # print(gait.joints)
        gait._send_angles()
        time.sleep(interval)
  except KeyboardInterrupt:
    pass

  gait.close()


def interactive(args=None):
  gait = FowardWaveNode(args, interpolation_steps=5, interpolation_wait=0.005)

  while True:
    options = (
      '[h] Home',
      '[j] Single Joint Control',
      '[a] Adjust Interpolation',
      '[x] Exit'
    )
    menu = TerminalMenu(options, title='Robot Interactive Control Menu')
    choice_idx = menu.show()

    if choice_idx == 0:
      gait.joints = {
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
      gait.send_angles()

    elif choice_idx == 1:
      while True:
        key, _, value = input('Enter [JointID] [Value]: ').partition(' ')
        if key.lower() == 'all':
          for jkey in gait.joints.keys():
            gait.joints[jkey] = float(value)
        elif key.lower() == 'exit':
          break
        else:
          gait.joints[key] = float(value)
        gait.send_angles()

    elif choice_idx == 2:
      print(f'Current Interpolation Steps: {gait._inter_steps}')
      print(f'Current Interpolation Wait Time: {gait._inter_wait}')

      gait._inter_steps = int(input('New Interpolation Steps: '))
      gait._inter_wait = float(input('New Interpolation Wait Time: '))

    elif choice_idx == len(options) - 1:
      break