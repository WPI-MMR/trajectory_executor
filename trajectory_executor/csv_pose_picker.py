import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, JointAngles

import math
from trajectory_executor.Parameters import *
import numpy as np
import csv


class CSVPosePicker(Node):
  """Sends a single pose chosen from a CSV file"""
  def __init__(self):
    super().__init__('csv_pose_picker')
    self._publisher = self.create_publisher(
      Trajectory,
      'new_traj',
      10)

    # reply = Trajectory()
    # reply.data = 'This will contain new trajectory data'
    # self._publisher.publish(reply)

def main(args=None):
  rclpy.init(args=args)
  csv_pose_picker = CSVPosePicker()

  traj = Trajectory()
  traj.traj = read_csv()
  csv_pose_picker._publisher.publish(traj)

  rclpy.shutdown()

def read_csv():
  with open('standing_traj_v1.csv', 'r') as infile:
    csvreader = csv.reader(infile)
    fields_indices = {}
    fields = next(csvreader)
    for i in range(len(fields)):
      fields_indices[fields[i]] = i

    ja_setpoints = []
    raw_setpoint = []
    for row in csvreader:
      raw_setpoint.append(row)

    setpoint = raw_setpoint[60]

    ja = JointAngles()
    ja.right_shoulder = invert_ja(int(float(setpoint[fields_indices['r_shoulder']])))
    ja.right_elbow = invert_ja(int(float(setpoint[fields_indices['r_elbow']])))
    ja.right_hip = int(float(setpoint[fields_indices['r_hip']]))
    ja.right_knee = int(float(setpoint[fields_indices['r_knee']]))
    ja.left_hip = invert_ja(int(float(setpoint[fields_indices['l_hip']])))
    ja.left_knee = invert_ja(int(float(setpoint[fields_indices['l_knee']])))
    ja.left_shoulder = int(float(setpoint[fields_indices['l_shoulder']]))
    ja.left_elbow = int(float(setpoint[fields_indices['l_elbow']]))
    ja.left_ankle = int(float(setpoint[fields_indices['l_ankle']]))+90
    ja.right_ankle = int(float(setpoint[fields_indices['r_ankle']]))+90
    ja_setpoints.append(ja)

    print(len(raw_setpoint))
    return ja_setpoints

def invert_ja(ja):
  return 0 - ja + 360

if __name__ == '__main__':
  main()
