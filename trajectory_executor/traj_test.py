import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles, Trajectory

import random

class TrajTest(Node):
  """Tests sending a trajectory to the tracker node for execution"""
  def __init__(self):
    super().__init__('traj_test')

    self.publisher = self.create_publisher(
      Trajectory,
      'trajectory',
      10)


def main(args=None):
  rclpy.init(args=args)
  traj_tester = TrajTest()

  traj = Trajectory()

  traj_tester.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()