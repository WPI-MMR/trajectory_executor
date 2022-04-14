import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, SensorData, JointAngles

import math
from trajectory_executor.Parameters import *
import numpy as np


class QuadWalkGenerator(Node):
  """Trajectory generation for a defined 8-DOF quadruped robot"""
  def __init__(self):
    super().__init__('quad_walk_generator')
    self._publisher = self.create_publisher(
      Trajectory,
      'new_traj',
      10)
    self._subscription = self.create_subscription(
      SensorData,
      'traj_start_data',
      self.create_trajectory_callback,
      10)

  def create_trajectory_callback(self, msg: SensorData):
    """Receive sensor data and publish trajectory to '/new_traj'"""
    self.get_logger().info('Received: {}'.format(msg.data))

    # reply = Trajectory()
    # reply.data = 'This will contain new trajectory data'
    # self._publisher.publish(reply)

def main(args=None):
  rclpy.init(args=args)
  quad_walk_generator = QuadWalkGenerator()

  traj = Trajectory()
  traj.traj = walk_cycle(5)
  quad_walk_generator._publisher.publish(traj)

  rclpy.shutdown()

def walk_cycle(num_cycles):
  start_x = 250
  start_y = 50
  y_offset = 45 # mm
  cycle_time = 8
  subcycle_time = cycle_time / 4

  seq1_x = [(start_x - 80*(math.sin(i*math.pi/25))) for i in range(0, 25)]
  seq1_y = [i + y_offset for i in range(start_y*2, -start_y*2, -8)]
  seq1 = list(zip(seq1_x, seq1_y))

  seq2_x = [start_x for i in range(-start_y*3, start_y*3, 4)]
  seq2_y = [i/1.5 + y_offset for i in range(-start_y*3, start_y*3, 4)]
  seq2 = list(zip(seq2_x, seq2_y))

  cycle1 = (seq1 + seq2)
  offset = int(len(cycle1) / 4)
  cycle2 = cycle1[offset:] + cycle1[:offset]
  cycle3 = cycle2[offset:] + cycle2[:offset]
  cycle4 = cycle3[offset:] + cycle3[:offset]

  ja_setpoints = []
  for i in range(len(cycle1)):
    ja = JointAngles()
    ja.right_shoulder = invert_ja(ik_solver(cycle1[i])[0])
    ja.right_elbow = invert_ja(ik_solver(cycle1[i])[1])
    ja.right_hip = invert_ja(rear_ik_solver(cycle2[i])[0])
    ja.right_knee = invert_ja(rear_ik_solver(cycle2[i])[1])
    ja.left_hip = rear_ik_solver(cycle4[i])[0]
    ja.left_knee = rear_ik_solver(cycle4[i])[1]
    ja.left_shoulder = ik_solver(cycle3[i])[0]
    ja.left_elbow = ik_solver(cycle3[i])[1]
    ja.left_ankle = 0
    ja.right_ankle = 0

    ja_setpoints.append(ja)

  return ja_setpoints*num_cycles

def ik_solver(coord):
  ja = JointAngles()
  print(coord)
  x = coord[0]
  y = coord[1]

  p = x**2 + y**2 - l12**2 - l22**2
  q = 2 * l12 * l22

  a = p / q
  if a > 1:
    a = 1
  elif a < -1:
    a = -1

  theta22 = -math.acos(a)

  k = (l22 * math.sin(theta22)) / (l12 + l22 * math.cos(theta22))
  theta12 = math.atan(y / x) - math.atan(k)

  theta12 = theta12 * 180 / math.pi
  theta22 = theta22 * 180 / math.pi

  if theta12 < 0:
    theta12 += 360
  if theta22 < 0:
    theta22 += 360

  return (int(theta12), int(theta22))

def rear_ik_solver(coord):
  ja = JointAngles()
  print(coord)
  x = coord[0]
  y = coord[1]

  p = x**2 + y**2 - l12**2 - l22**2
  q = 2 * l12 * l22

  a = p / q
  if a > 1:
    a = 1
  elif a < -1:
    a = -1

  theta22 = math.acos(a)

  k = (l22 * math.sin(theta22)) / (l12 + l22 * math.cos(theta22))
  theta12 = math.atan(y / x) - math.atan(k)

  theta12 = theta12 * 180 / math.pi
  theta22 = theta22 * 180 / math.pi

  if theta12 < 0:
    theta12 += 360
  if theta22 < 0:
    theta22 += 360

  return (int(theta12), int(theta22))


def invert_ja(ja):
  return 0 - ja + 360

if __name__ == '__main__':
  main()
