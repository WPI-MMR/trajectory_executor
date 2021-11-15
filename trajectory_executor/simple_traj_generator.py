import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, SensorData, JointAngles

import math
from trajectory_executor.Parameters import *
import numpy as np


class SimpleTrajectoryGenerator(Node):
  """Trajectory generation for a defined 8-DOF quadruped robot"""
  def __init__(self):
    super().__init__('simple_traj_generator')
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
  simple_traj_generator = SimpleTrajectoryGenerator()

  traj = Trajectory()
  traj.traj = custom_move()
  simple_traj_generator._publisher.publish(traj)

  rclpy.shutdown()

def line_move():
  x_vals = [i for i in range(100, 150, 5)]
  y_vals = [x for x in x_vals]
  ja_setpoints = [ik_solver(coord) for coord in zip(x_vals, y_vals)]
  return ja_setpoints

def custom_move():
  # Traj 1
  x1 = []
  x1 = np.arange(333.5, 270, -1).tolist()
  x1_len = len(x1)
  y1 = np.zeros(x1_len)
  y1 = y1.tolist()
  traj1 = [x1, y1]

  # Traj 2
  y2 = np.arange(0, 50, 1).tolist()
  x2 = []
  for i in y2:
      x2_temp = 15 * math.sin(5 * i)
      x2_temp = x2_temp + 270
      x2.append(x2_temp)
  traj2 = [x2, y2]

  #Traj 3
  y3 = np.arange(50, 150, 1).tolist()
  x2_len = len(x2)
  y3_len = len(y3)
  x3 = x2[x2_len - 1]
  x3_temp = np.ones(y3_len)
  x3_temp = x3_temp.tolist()
  x3 = [i * x2[x2_len-1] for i in x3_temp]
  traj3 = [x3, y3]

  #Traj 4
  y4 = np.arange(150, 200, 1).tolist()
  x3_len = len(x3)
  x4 = []
  for i in y2:
      x4_temp = -15 * math.sin(5 * i)
      x4_temp = x4_temp + x3[x3_len-1]
      x4.append(x4_temp)
  traj4 = [x4, y4]

  #Traj 5
  x4_len = len(x4)
  x5 = np.arange(x4[x4_len -1], 333.5, 1).tolist()
  x5_len = len(x5)
  y4_len = len(y4)
  y5_temp = np.ones(x5_len).tolist()
  y5 = [i * y4[y4_len-1] for i in y5_temp]
  traj5 = [x5, y5]

  # Combine Traj
  traj_x = traj1[0] + traj2[0] + traj3[0] + traj4[0] + traj5[0]
  traj_y = traj1[1] + traj2[1] + traj3[1] + traj4[1] + traj5[1]
  traj = [traj_x, traj_y]

  # print([coord for coord in zip(traj_x, traj_y)])

  ja_setpoints = [ik_solver(coord) for coord in zip(traj_x, traj_y)]
  return ja_setpoints

def arc_move():
  pass

def complex_move():
  pass


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
