import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, SensorData, JointAngles


class SimpleTrajectoryGenerator(Node):
  """Trajectory generation for a defined 8-DOF quadruped robot"""
  def __init__(self):
    super().__init__('simple_traj_generator')
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
  simple_traj_generator = SimpleTrajectoryGenerator()

  traj = Trajectory()
  traj.traj = line_move()
  simple_traj_generator._publisher.publish(traj)

  rclpy.shutdown()

def line_move():
  x_vals = [i for i in range(10)]
  y_vals = [x for x in x_vals]
  ja_setpoints = [ik_solver(coord) for coord in zip(x_vals, y_vals)]
  return ja_setpoints

def arc_move():
  pass

def complex_move():
  pass


def ik_solver(coord):
  ja = JointAngles()

  # calculations

  return ja

if __name__ == '__main__':
  main()
