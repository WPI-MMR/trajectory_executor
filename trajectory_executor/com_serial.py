import rclpy
from rclpy.node import Node

# from trajectory_interfaces.srv import SensorDataRequest, JointAngles
from trajectory_interfaces.msg import JointAngles, AckResponse, DataRequest, DataResponse

import serial
import struct
import copy

from trajectory_executor.datastructures import serial_read_states, data_packet_struct, ack_packet_struct


class SerialConnection(Node):
  """Manages a serial connection between ROS and Arduino"""
  def __init__(self):
    super().__init__('serial_connection')

    self.preamble_length = 4
    self.data_byte_length = 2
    self.port_in_use = False

    self.dr_publisher = self.create_publisher(
      DataResponse,
      'data_response',
      10)

    self.ja_publisher = self.create_publisher(
      AckResponse,
      'ack_response',
      10)

    self.dr_subscription = self.create_subscription(
      DataRequest,
      'data_request',
      self.sensor_data_request_callback,
      10)

    self.ja_subscription = self.create_subscription(
      JointAngles,
      'joint_angles',
      self.send_joint_angles_callback,
      10)

    # self.data_service = self.create_service(
    #   SensorDataRequest,
    #   'sensor_data_request',
    #   self.sensor_data_request_callback
    # )

    # self.ja_service = self.create_service(
    #   JointAngles,
    #   'joint_angles',
    #   self.send_joint_angles_callback,
    # )

    self.arduino_port = serial.Serial(
      port="/dev/ttyAMA1",  # TODO: evaluate if we should change to cli flag
      baudrate=115200
    )

  def sensor_data_request_callback(self, msg: DataRequest):
    """Ping Arduino for sensor data"""
    # self.get_logger().info("Requesting robot state")

    while self.port_in_use:
      # another serial operation is in progress... wait for completion to continue
      pass

    # claim the serial port to prevent interference
    self.port_in_use = True

    request_sensors = 1
    checksum = 255 - request_sensors % 256

    # send request for sensor data
    # the '>B' parameter tells struct.pack how to encode the data
    # (in this case encode into bytes)
    for i in range(self.preamble_length):
      self.arduino_port.write(struct.pack('>B', 255))
    self.arduino_port.write(struct.pack('>B', request_sensors))
    self.arduino_port.write(struct.pack('>B', checksum))

    ser_in = self.read_ser()

    data = DataResponse()
    data.roll = ser_in['roll']
    data.pitch = ser_in['pitch']
    data.yaw = ser_in['yaw']
    data.l_hip = ser_in['l_hip']
    data.l_knee = ser_in['l_hip']
    data.r_hip = ser_in['r_hip']
    data.r_knee = ser_in['r_hip']
    data.l_shoulder = ser_in['l_shoulder']
    data.l_elbow = ser_in['l_elbow']
    data.r_shoulder = ser_in['r_shoulder']
    data.r_elbow = ser_in['r_elbow']
    data.at_goal = ser_in['at_goal']

    self.dr_publisher.publish(data)

    # release serial port for other processes to use
    self.port_in_use = False
    # return response

  def send_joint_angles_callback(self, msg: JointAngles):
    """Send new joint angles to Arduino for execution"""
    # self.get_logger().info("Sending joint angles")

    while self.port_in_use:
      # another serial operation is in progress... wait for completion to continue
      pass

    # claim the serial port to prevent interference
    self.port_in_use = True

    # Split joint angles into two numbers since max value of a byte is 255
    ja_bytes = []
    ja_bytes.append(0) # data request byte
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
    # the '>B' parameter tells struct.pack how to encode the data
    # (in this case encode into bytes)
    for i in range(self.preamble_length):
      self.arduino_port.write(struct.pack('>B', 255))
    for ja_byte in ja_bytes:
      self.arduino_port.write(struct.pack('>B', ja_byte))

    ser_in = self.read_ser()

    ack = AckResponse()
    ack.setpoint_ack = ser_in['setpoint_ack']
    self.ja_publisher.publish(ack)

    # release serial port for other processes to use
    self.port_in_use = False
    # return response

  def read_ser(self):
    # prep to read serial data
    read_state = serial_read_states['READ_PREAMBLE']
    preamble_counter = self.preamble_length
    data_byte_counter = self.data_byte_length
    calculated_checksum = 0

    # simple deepcopy for dicts with primitives datatypes
    data_packet = copy.deepcopy(data_packet_struct)
    read_corrupted = False

    # read in sensor data from serial port
    while rclpy.ok():
      if self.arduino_port.in_waiting > 0:
        # read and decode data from serial buffer
        read_corrupted = False
        recv_data = self.arduino_port.read(1)
        decoded_data = int(recv_data.hex(), 16)

        # run through state machine
        if read_state == serial_read_states['READ_PREAMBLE']:
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
          read_state = serial_read_states['READ_ACK']
        elif read_state == serial_read_states['READ_ACK']:
          data_packet['setpoint_ack'] = decoded_data
          calculated_checksum += decoded_data
          read_state = serial_read_states['READ_CHECKSUM']
        elif read_state == serial_read_states['READ_CHECKSUM']:
          data_packet['checksum'] = decoded_data
          if data_packet['checksum'] == 255 - (calculated_checksum % 256):
            # good packet, relay to tracking node
            break
          else:
            # bad packet, retry request
            # return self.sensor_data_request_callback(request, response)
            internal_checksum = 255 - (calculated_checksum % 256)
            self.get_logger().info(f"Bad request: Received checksum {data_packet['checksum']} but expected {internal_checksum}")
            break

      else:
        if read_corrupted:
          # self.get_logger().info("Read corrupted. Aborting")
          break
        else:
          read_corrupted = True # gives a second chance in case we just missed a beat

    return data_packet





def main(args=None):
  rclpy.init(args=args)
  serial_con = SerialConnection()

  rclpy.spin(serial_con)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
