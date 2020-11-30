import rclpy
from rclpy.node import Node


class SocketConnection(Node):
  """Manages a socket connection between ROS and PyBullet wrapper"""
  def __init__(self):
      super().__init__('socket_connection')

def main(args=None):
  rclpy.init(args=args)
  socket_con = SocketConnection()

  rclpy.spin(socket_con)
  rclpy.shutdown()

if __name__ == "__main__":
  main()