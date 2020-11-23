import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, SensorData


class TrajectoryGenerator(Node):
  def __init__(self):
    super().__init__('trajectory_generator')
    self._publisher = self.create_publisher(
      Trajectory,
      'new_traj',
      10)
    self._subscription = self.create_subscription(
      SensorData,
      'traj_start_data',
      self.create_trajectory_callback,
      10)

  def create_trajectory_callback(self, msg):
    self.get_logger().info('Received: {}'.format(msg.data))

    reply = Trajectory()
    reply.data = 'This will contain new trajectory data'
    self._publisher.publish(reply)

def main(args=None):
  rclpy.init(args=args)
  trajectory_generator = TrajectoryGenerator()

  rclpy.spin(trajectory_generator)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
