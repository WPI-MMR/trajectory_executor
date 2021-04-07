import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles, Trajectory
from trajectory_interfaces.srv import SensorDataRequest

import serial


class TrajectoryTracker(Node):
  """Trajectory tracking for an 8-DOF quadruped robot given a trajectory"""
  def __init__(self):
    super().__init__('trajectory_tracker')
    self.ja_pub = self.create_publisher(
      JointAngles,
      'joint_angles',
      10)

    self.traj_sub = self.create_subscription(
      Trajectory,
      'trajectory',
      self.traj_callback,
      10)

    self.srv_client = self.create_client(
      SensorDataRequest,
      'sensor_data_request')

    while not self.srv_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info("Serial service inactive, waiting...")

    self.req = SensorDataRequest.Request()

  def send_data_request(self):
    self.req.requested = True
    self.future = self.srv_client.call_async(self.req)

  def traj_callback(self, traj: Trajectory):
    """Receive trajectory from generator node"""
    self.get_logger.info("Trajectory received. Executing...")
    for ja in traj:
      self.ja_pub.publish(ja)
      goal = False

      while rclpy.ok():
        self.send_data_request()
        while rclpy.ok():
          # rclpy.spin_once(self)
          if self.future.done():
            try:
              response = self.future.result()
            except Exception as e:
              self.get_logger().warn(f"Service call failed {e}")
            else:
              if response.at_goal:
                goal = True
              break

        if goal:
          break


def main(args=None):
  rclpy.init(args=args)
  trajectory_tracker = TrajectoryTracker()

  rclpy.spin(trajectory_tracker)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
