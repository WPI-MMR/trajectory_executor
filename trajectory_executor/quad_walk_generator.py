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
  cycle_time = 8
  subcycle_time = cycle_time / 4

  seq1_x = [(start_x - 80*(math.sin(i*math.pi/50))) for i in range(0, 50)]
  seq1_y = [i for i in range(start_y, -start_y, -2)]
  seq1 = [coord for coord in zip(seq1_x, seq1_y)]

  seq2_x = [start_x for i in range(-start_y*2, start_y*2)]
  seq2_y = [i/2 for i in range(-start_y*2, start_y*2)]
  seq2 = [coord for coord in zip(seq2_x, seq2_y)]

  cycle = (seq1 + seq2)*num_cycles
  ja_setpoints = [ik_solver(coord) for coord in cycle]
  return ja_setpoints

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


  ja.right_shoulder = int(theta12)
  ja.right_elbow = int(theta22)
  ja.left_hip = 0
  ja.left_knee = 0
  ja.right_hip = 0
  ja.right_knee = 0
  ja.left_shoulder = 0
  ja.left_elbow = 0

  return ja

if __name__ == '__main__':
  main()
