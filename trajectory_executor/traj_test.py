import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles, Trajectory

import random
import time

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
  traj_tester.get_logger().info("Waiting 2 seconds")
  time.sleep(2)

  l_hip_angle = 0
  l_knee_angle = 0

  # Generate joint angles and fill trajectory object
  for i in range(num_ja):
    if i % 2 == 0:
      l_hip_angle += 45
      l_knee_angle -= 45
      if l_hip_angle >= 360:
        l_hip_angle -= 360
      if l_knee_angle < 0:
        l_knee_angle += 360
    else:
      l_hip_angle -= 90
      l_knee_angle += 90
      if l_hip_angle < 0:
        l_hip_angle += 360      
      if l_knee_angle >= 360:
        l_knee_angle -= 360

    angles = JointAngles()

    angles.left_hip = l_hip_angle
    angles.left_knee = l_knee_angle
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
  # time.sleep(1)

  traj_tester.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()