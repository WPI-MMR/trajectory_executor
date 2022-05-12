import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, SensorData, JointAngles


class MoveHome(Node):
  """Trajectory generation for a defined 8-DOF quadruped robot"""
  def __init__(self):
    super().__init__('move_home')
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
  move_home = MoveHome()

  home_ja = JointAngles()
  home_ja.right_shoulder = 0
  home_ja.right_elbow = 0
  home_ja.left_hip = 0
  home_ja.left_knee = 0
  home_ja.right_hip = 0
  home_ja.right_knee = 0
  home_ja.left_shoulder = 0
  home_ja.left_elbow = 0
  home_ja.left_ankle = 120
  home_ja.right_ankle = 120

  traj = Trajectory()
  traj.traj = [home_ja]

  move_home._publisher.publish(traj)

  rclpy.shutdown()
