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

  num_ja = 10
  traj = Trajectory()

  # Generate joint angles and fill trajectory object
  for i in range(num_ja):
    angles = JointAngles()

    angles.left_hip = random.randint(0, 359)
    angles.left_knee = random.randint(0, 359)
    # angles.right_hip = random.randint(0, 359)
    # angles.right_knee = random.randint(0, 359)
    # angles.left_shoulder = random.randint(0, 359)
    # angles.left_elbow = random.randint(0, 359)
    # angles.right_shoulder = random.randint(0, 359)
    # angles.right_elbow = random.randint(0, 359)
    angles.right_hip = 0
    angles.right_knee = 0
    angles.left_shoulder = 0
    angles.left_elbow = 0
    angles.right_shoulder = 0
    angles.right_elbow = 0

    traj.trajectory.append(angles)

  # Publish new trajectory to ros topic
  traj_tester.publisher.publish(traj)

  traj_tester.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()