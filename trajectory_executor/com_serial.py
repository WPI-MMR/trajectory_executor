import rclpy
from rclpy.node import Node

from trajectory_interfaces.srv import SensorDataRequest
from trajectory_interfaces.msg import JointAngles

import serial
import struct


class SerialConnection(Node):
  """Manages a serial connection between ROS and Arduino"""
  def __init__(self):
    super().__init__('serial_connection')

    self.preamble_length = 4

    self._service = self.create_service(
      SensorDataRequest,
      'sensor_data_request',
      self.sensor_data_request_callback
    )

    self._subscriber = self.create_subscription(
      JointAngles,
      'joint_angles',
      self.send_joint_angles_callback,
      10
    )

    self.arduino_port = serial.Serial(
      port="/dev/ttyAMA1",  # TODO: evaluate if we should change to cli flag
      baudrate=115200
    )

  def sensor_data_request_callback(self, request: SensorDataRequest, response: SensorDataRequest):
    """Ping Arduino for sensor data"""
    request_sensors = 1
    checksum = 255 - request_sensors

    for i in range(self.preamble_length):
      self.arduino_port.write(struct.pack('>B', 255))

    self.arduino_port.write(struct.pack('>B', request_sensors))
    self.arduino_port.write(struct.pack('>B', checksum))

    # wait for serial response here

    return response

  def send_joint_angles_callback(self, msg: JointAngles):
    """Send new joint angles to Arduino for execution"""
    pass


def main(args=None):
  rclpy.init(args=args)
  serial_con = SerialConnection()

  rclpy.spin(serial_con)
  rclpy.shutdown()

if __name__ == "__main__":
  main()