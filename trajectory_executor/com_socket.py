import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SocketConnection(Node):
  """Manages a socket connection between ROS and PyBullet wrapper"""
  def __init__(self):
      super().__init__('socket_connection')
      self.subscription = self.create_subscription(
          String,
          'joint_angles',
          self.listener_callback,
          10)
      self.subscription  # prevent unused variable warning

  def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
  rclpy.init(args=args)
  socket_con = SocketConnection()

  rclpy.spin(socket_con)
  socket_con.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()