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

    # TODO: Make this into a ros parameter
    self.virtual_serial_comm = serial.Serial('/dev/pts/2')

  def listener_callback(self, msg):
    try:
      # Try to import the EOL from other package 
      self.virtual_serial_comm.write(msg.data.encode() + b'\r\n')
    except SerialException:
      # TODO: Change this to error level in a seperate commit
      self.get_logger().error('Cannot use serial port.' \
       ' Check if dummy serial port is active,' \
       ' serial port number is correct and restart the program')
    else:
      self.get_logger().info('Passing along: "%s"' % msg.data)

      res = b''
      while not res.endswith(b'\r\n'):
        res += self.virtual_serial_comm.read()
      
      print('Response: {}'.format(res))    


def main(args=None):
  rclpy.init(args=args)
  socket_con = SocketConnection()

  rclpy.spin(socket_con)
  socket_con.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()