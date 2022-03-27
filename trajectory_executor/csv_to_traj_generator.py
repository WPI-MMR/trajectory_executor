import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, JointAngles

import math
from trajectory_executor.Parameters import *
import numpy as np
import csv


class CSVToTrajGenerator(Node):
  """Grab and send a trajectory from a CSV file"""
  def __init__(self):
    super().__init__('csv_to_traj_generator')
    self._publisher = self.create_publisher(
      Trajectory,
      'new_traj',
      10)

    # reply = Trajectory()
    # reply.data = 'This will contain new trajectory data'
    # self._publisher.publish(reply)

def main(args=None):
  rclpy.init(args=args)
  csv_to_traj_generator = CSVToTrajGenerator()

  traj = Trajectory()
  traj.traj = read_csv()
  csv_to_traj_generator._publisher.publish(traj)

  rclpy.shutdown()

def read_csv():
  with open('standing_traj.csv', 'r') as infile:
    csvreader = csv.reader(infile)
    fields_indices = {}
    fields = next(csvreader)
    for i in range(len(fields)):
      fields_indices[fields[i]] = i

    ja_setpoints = []
    for row in csvreader:
      ja = JointAngles()
      ja.right_shoulder = invert_ja(int(float(row[fields_indices['r_shoulder']])))
      ja.right_elbow = invert_ja(int(float(row[fields_indices['r_elbow']])))
      ja.right_hip = int(float(row[fields_indices['r_hip']]))
      ja.right_knee = int(float(row[fields_indices['r_knee']]))
      ja.left_hip = invert_ja(int(float(row[fields_indices['l_hip']])))
      ja.left_knee = invert_ja(int(float(row[fields_indices['l_knee']])))
      ja.left_shoulder = int(float(row[fields_indices['l_shoulder']]))
      ja.left_elbow = int(float(row[fields_indices['l_elbow']]))
      ja.left_ankle = 1
      ja.right_ankle = 1
      ja_setpoints.append(ja)

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

def invert_ja(ja):
  return 0 - ja + 360

if __name__ == '__main__':
  main()
