import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial
from serial import SerialException
import json


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
    self.virtual_serial_comm = serial.Serial('/dev/pts/5')

  def listener_callback(self, msg):
    try: 
      self.virtual_serial_comm.write((msg.data + '\n').encode())
    except SerialException:
      # TODO: Change this to error level in a seperate commit
      self.get_logger().info('Cannot use serial port. Check if dummy serial port is active, serial port number is correct and restart the program')
    else:
      self.get_logger().info('Passing along: "%s"' % msg.data)


def main(args=None):
  rclpy.init(args=args)
  socket_con = SocketConnection()

  rclpy.spin(socket_con)
  socket_con.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()