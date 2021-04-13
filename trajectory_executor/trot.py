import rclpy 
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest

import time
import numpy as np


def _set_angle_FOR(angle):
  if angle < 0:
    angle = 360 + angle
  return int(angle)


QUAD_STANDING = {
  'FL_HFE': 45,
  'FL_KFE': -100,
  'FL_ANKLE': 0,
  'FR_HFE': 45,
  'FR_KFE': -100,
  'FR_ANKLE': 0,
  'HL_HFE': 25,
  'HL_KFE': -60,
  'HL_ANKLE': 0,
  'HR_HFE': 25,
  'HR_KFE': -60,
  'HR_ANKLE': 0,
}



class Trot(Node):
  def __init__(self) -> None:
    super().__init__('trot')

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

    # Create publisher
    self.joint_publisher = self.create_publisher(
      JointAngles, 'joint_angles', 10)

    # Create to ensure that the connection is up with com_serial. Not actually 
    # used.
    self.sensors_client = self.create_client(
      SensorDataRequest, 'sensor_data_request')

    while not self.sensors_client.wait_for_service(timeout_sec=1.):
      self.get_logger().warning('Serial service inactive, waiting...')

  def send_angles(self):
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


def trot(args=None):
  """Run a super rudamentary, hardcoded trot in quadrupedal mode"""
  rclpy.init(args=args)
  trot = Trot()

  # Initialize robot position
  trot.joints = {
    'FL_HFE': 45,
    'FL_KFE': -90,
    'FL_ANKLE': 0,
    'FR_HFE': 45,
    'FR_KFE': -90,
    'FR_ANKLE': 0,
    'HL_HFE': 45,
    'HL_KFE': -90,
    'HL_ANKLE': 0,
    'HR_HFE': 45,
    'HR_KFE': -90,
    'HR_ANKLE': 0,
  }
  trot.send_angles()
  input('waiting')

  # Robot configuration values
  trot_hip_launch = -0.8 * -180 / np.pi
  trot_knee_launch = 1.4 * -180 / np.pi
  trot_launch_dur = 0.25 * -180 / np.pi
  trot_knee_clearance = 2 * -180 / np.pi
  trot_clearance_dur = 0.1 * -180 / np.pi
  trot_hip_step = -0.05 * -180 / np.pi
  trot_knee_step = 1.5 * -180 / np.pi
  trot_step_dur = 0.1 * -180 / np.pi

  # while True:
  # Get ready to launch FR and HL
  trot.FLHR_HFE(trot_hip_launch)
  trot.FLHR_KFE(trot_knee_launch)
  trot.FRHL_KFE(trot_knee_clearance)
  trot.send_angles()
  time.sleep(trot_launch_dur)
  
  """

  # Make the FR and HL Movement
  trot.FRHL_HFE(trot_hip_step)
  trot.FRHL_KFE(trot_knee_step)
  trot.send_angles()
  time.sleep(trot_step_dur)

  # Get ready to launch FL and HR
  trot.FRHL_HFE(trot_hip_launch)
  trot.FRHL_KFE(trot_knee_launch)
  trot.FLHR_KFE(trot_knee_clearance)
  trot.send_angles()
  time.sleep(trot_launch_dur)

  # Make the FL and HR Movement
  trot.FLHR_HFE(trot_hip_step)
  trot.FLHR_KFE(trot_knee_step)
  trot.send_angles()
  time.sleep(trot_step_dur)
  """
  
  trot.destroy_node()
  rclpy.shutdown()


def calibrate(args=None):
  """Offer the robotist an interactive terminal to manually set the joint
  values."""

  rclpy.init(args=args)
  trot = Trot()
  
  # trot.joints = {
  #   'FL_HFE': 45,
  #   'FL_KFE': 90,
  #   'FL_ANKLE': 0,
  #   'FR_HFE': -45,
  #   'FR_KFE': -90,
  #   'FR_ANKLE': 0,
  #   'HL_HFE': 45,
  #   'HL_KFE': -90,
  #   'HL_ANKLE': 0,
  #   'HR_HFE': -45,
  #   'HR_KFE': -90,
  #   'HR_ANKLE': 0,
  # }
  trot.joints = {
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
  trot.send_angles()

  while True:
    try:
      key, _, value = input('Enter [JointID] [Value]: ').partition(' ')
      if key.lower() == 'all':
        for jkey in trot.joints.keys():
          trot.joints[jkey] = float(value)
      elif key.lower() == 'quad':
        steps, _ , timing = value.partition(' ')

        for i in range(int(steps)):
          for k, v in QUAD_STANDING.items():
            trot.joints[k] = int(v / int(steps) * (i + 1))
          trot.send_angles()
          time.sleep(float(timing))
      elif key.lower() == 'new':
        trot.joints = {
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
        trot.send_angles()
      else:
        trot.joints[key] = float(value)
      trot.send_angles()

    except KeyboardInterrupt:
      break

  trot.destroy_node()
  rclpy.shutdown()
