import rclpy
from rclpy.node import Node

from trajectory_interfaces.srv import SensorDataRequest
from trajectory_interfaces.msg import JointAngles

import serial
import struct
import json

from datastructures import serial_read_states, data_packet_struct


class SerialConnection(Node):
  """Manages a serial connection between ROS and Arduino"""
  def __init__(self):
    super().__init__('serial_connection')

    self.preamble_length = 4
    self.data_byte_length = 2

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
    checksum = 255 - (request_sensors * self.data_byte_length) % 256

    # send request for sensor data
    for i in range(self.preamble_length):
      self.arduino_port.write(struct.pack('>B', 255))
    for i in range(self.data_byte_length):
      self.arduino_port.write(struct.pack('>B', request_sensors))
    self.arduino_port.write(struct.pack('>B', checksum))

    # prep to read serial data
    read_state = serial_read_states['INIT']
    preamble_counter = self.preamble_length
    data_byte_counter = self.data_byte_length
    calculated_checksum = 0

    # simple deepcopy for dicts with primitives datatypes
    data_packet = json.loads(json.dumps(data_packet_struct))

    # read in sensor data from serial port
    while True:
      if self.arduino_port.in_waiting > 0:
        # read and decode data from serial buffer
        recv_data = self.arduino_port.read(1)
        decoded_data = int(received_data.hex(), 16)

        # run through state machine
        if read_state == serial_read_states['INIT']:
          calculated_checksum = 0
          read_state = serial_read_states['READ_PREAMBLE']
        elif read_state == serial_read_states['READ_PREAMBLE']:
          if decoded_data == 255:
            preamble_counter -= 1
            if preamble_counter == 0:
              preamble_counter = self.preamble_length
              read_state = serial_read_states['READ_ROLL']
          else:
            preamble_counter = self.preamble_length
        elif read_state == serial_read_states['READ_ROLL']:
          data_packet['roll'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_PITCH']
        elif read_state == serial_read_states['READ_PITCH']:
          data_packet['pitch'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_YAW']
        elif read_state == serial_read_states['READ_YAW']:
          data_packet['yaw'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_L_HIP']
        elif read_state == serial_read_states['READ_L_HIP']:
          data_packet['l_hip'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_L_KNEE']
        elif read_state == serial_read_states['READ_L_KNEE']:
          data_packet['l_knee'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_R_HIP']
        elif read_state == serial_read_states['READ_R_HIP']:
          data_packet['r_hip'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_R_KNEE']
        elif read_state == serial_read_states['READ_R_KNEE']:
          data_packet['r_knee'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_L_SHOULDER']
        elif read_state == serial_read_states['READ_L_SHOULDER']:
          data_packet['l_shoulder'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_L_ELBOW']
        elif read_state == serial_read_states['READ_L_ELBOW']:
          data_packet['l_elbow'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_R_SHOULDER']
        elif read_state == serial_read_states['READ_R_SHOULDER']:
          data_packet['r_shoulder'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_R_ELBOW']
        elif read_state == serial_read_states['READ_R_ELBOW']:
          data_packet['r_elbow'] += decoded_data
          calculated_checksum += decoded_data
          data_byte_counter -= 1
          if data_byte_counter == 0:
            data_byte_counter = self.data_byte_length
            read_state = serial_read_states['READ_AT_GOAL']
        elif read_state == serial_read_states['READ_AT_GOAL']:
          data_packet['at_goal'] = decoded_data
          calculated_checksum += decoded_data
          read_state = serial_read_states['READ_CHECKSUM']
        elif read_state == serial_read_states['READ_CHECKSUM']:
          data_packet['checksum'] = decoded_data
          if data_packet['checksum'] == 255 - (calculated_checksum % 256):
            # good packet, relay to tracking node
            response.roll = data_packet['roll']
            response.pitch = data_packet['pitch']
            response.yaw = data_packet['yaw']
            response.l_hip = data_packet['l_hip']
            response.l_knee = data_packet['l_hip']
            response.r_hip = data_packet['r_hip']
            response.r_knee = data_packet['r_hip']
            response.l_shoulder = data_packet['l_shoulder']
            response.l_elbow = data_packet['l_elbow']
            response.r_shoulder = data_packet['r_shoulder']
            response.r_elbow = data_packet['r_elbow']
            response.at_goal = data_packet['at_goal']
            break
          else:
            # bad packet, retry request
            return self.sensor_data_request_callback(request, response)

    return response

  def send_joint_angles_callback(self, msg: JointAngles):
    """Send new joint angles to Arduino for execution"""
    # Split joint angles into two numbers since max value of a byte is 255
    ja_bytes = []
    ja_bytes.append(255 if msg.left_hip // 256 > 0 else msg.left_hip)
    ja_bytes.append(msg.left_hip % 255 if msg.left_hip // 256 > 0 else 0)
    ja_bytes.append(255 if msg.left_knee // 256 > 0 else msg.left_knee)
    ja_bytes.append(msg.left_knee % 255 if msg.left_knee // 256 > 0 else 0)
    ja_bytes.append(255 if msg.right_hip // 256 > 0 else msg.right_hip)
    ja_bytes.append(msg.right_hip % 255 if msg.right_hip // 256 > 0 else 0)
    ja_bytes.append(255 if msg.right_knee // 256 > 0 else msg.right_knee)
    ja_bytes.append(msg.right_knee % 255 if msg.right_knee // 256 > 0 else 0)
    ja_bytes.append(255 if msg.left_shoulder // 256 > 0 else msg.left_shoulder)
    ja_bytes.append(msg.left_shoulder % 255 if msg.left_shoulder // 256 > 0 else 0)
    ja_bytes.append(255 if msg.left_elbow // 256 > 0 else msg.left_elbow)
    ja_bytes.append(msg.left_elbow % 255 if msg.left_elbow // 256 > 0 else 0)
    ja_bytes.append(255 if msg.right_shoulder // 256 > 0 else msg.right_shoulder)
    ja_bytes.append(msg.right_shoulder % 255 if msg.right_shoulder // 256 > 0 else 0)
    ja_bytes.append(255 if msg.right_elbow // 256 > 0 else msg.right_elbow)
    ja_bytes.append(msg.right_elbow % 255 if msg.right_elbow // 256 > 0 else 0)

    # calculate checksum
    checksum = 255 - sum(ja_bytes) % 256
    ja_bytes.append(checksum)

    # send preamble, joint angles, and checksum
    for i in range(self.preamble_length):
      self.arduino_port.write(struct.pack('>B', 255))
    for ja_byte in ja_bytes:
      self.arduino_port.write(struct.pack('>B', ja_byte))



def main(args=None):
  rclpy.init(args=args)
  serial_con = SerialConnection()

  rclpy.spin(serial_con)
  rclpy.shutdown()

if __name__ == "__main__":
  main()