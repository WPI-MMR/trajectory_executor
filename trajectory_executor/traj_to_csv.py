import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, JointAngles

import csv

class TrajToCSV(Node):
  """Takes trajectories and outputs to a csv file"""
  def __init__(self):
    super().__init__("traj_to_csv")

    self.subscription = self.create_subscription(
      Trajectory,
      'new_traj',
      self.traj_callback,
      10)

  def traj_callback(self, traj: Trajectory):
    self.get_logger().info("Received trajectory")
    jas = traj.traj

    with open('trajectory.csv', 'w', newline='') as csvfile:
      file_writer = csv.writer(csvfile, delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)

      # add header row
      file_writer.writerow(['l_hip', 'l_knee', 'r_hip', 'r_knee', 'l_shoulder', 'l_elbow', 'r_shoulder', 'r_elbow'])

      # add data
      for ja in jas:
        file_writer.writerow(self.ja_to_list(ja))

    self.get_logger().info("Output complete")


  def ja_to_list(self, ja: JointAngles):
    outlist = []
    outlist.append(ja.left_hip)
    outlist.append(ja.left_knee)
    outlist.append(ja.right_hip)
    outlist.append(ja.right_knee)
    outlist.append(ja.left_shoulder)
    outlist.append(ja.left_elbow)
    outlist.append(ja.right_shoulder)
    outlist.append(ja.right_elbow)

    return outlist


def main(args=None):
  rclpy.init(args=args)
  traj_to_csv = TrajToCSV()

  rclpy.spin(traj_to_csv)
  rclpy.shutdown()