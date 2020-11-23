import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, SensorData

import serial


class TrajectoryTracker(Node):
  """Trajectory tracking for an 8-DOF quadruped robot given a trajectory"""
  def __init__(self):
    super().__init__('trajectory_tracker')
    self._publisher = self.create_publisher(
      SensorData,
      'traj_start_data',
      10)
    self._subscription = self.create_subscription(
      Trajectory,
      'new_traj',
      self.new_traj_callback,
      10)

    timer_period = 2 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.ard_check = self.create_timer(0.1, self.ard_check_callback)

    self.arduino_port = serial.Serial(
      port="/dev/ttyAMA1",  # TODO: evaluate if we should change to cli flag
      baudrate=115200
    )

  def ard_check_callback(self):
    """Check for serial data from Arduino"""
    if self.arduino_port.in_waiting:
      received_data = self.arduino_port.read(1)
      self.arduino_port.write(received_data)

  def timer_callback(self):
    """Publish Arduino sensor data to ROS topic '/traj_start_data'"""
    msg = SensorData()
    msg.data = 'Arduino Sensor Data'
    self._publisher.publish(msg)

  def new_traj_callback(self, msg: Trajectory):
    """Receive trajectory from generator node"""
    self.get_logger().info(msg.data)


def main(args=None):
  rclpy.init(args=args)
  trajectory_tracker = TrajectoryTracker()

  rclpy.spin(trajectory_tracker)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
