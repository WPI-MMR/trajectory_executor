import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest

import serial
import random


class SerialTest(Node):
  """Tests the serial communication pipeline"""
  def __init__(self):
    super().__init__('serial_test')

    self._publisher = self.create_publisher(
      JointAngles,
      'joint_angles',
      10)

    self._srv_client = self.create_client(
      SensorDataRequest,
      'sensor_data_request',
      callback_group=ReentrantCallbackGroup())

    while not self._srv_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service unavailable, trying again...')
    self.req = SensorDataRequest.Request()

    self.pub_timer = self.create_timer(2, self.pub_angles)
    self.srv_timer = self.create_timer(1, self.req_arduino_data)

  def pub_angles(self):
    angles = JointAngles()
    angles.left_hip = random.randint(0, 359)
    angles.left_knee = random.randint(0, 359)
    angles.right_hip = random.randint(0, 359)
    angles.right_knee = random.randint(0, 359)
    angles.left_shoulder = random.randint(0, 359)
    angles.left_elbow = random.randint(0, 359)
    angles.right_shoulder = random.randint(0, 359)
    angles.right_elbow = random.randint(0, 359)
    self._publisher.publish(angles)

  def req_arduino_data(self):
    self.req.requested = True
    self.future = self._srv_client.call_async(self.req)

    while rclpy.ok():
      # self.get_logger().info('weeeee')
      if self.future.done():
        try:
          response = self.future.result()
        except Exception as e:
          self.get_logger().info(
            'Service call failed %r' % (e,))
        else:
          self.get_logger().info("%d %d %d %d %d %d %d %d %d %d %d %d" % (
            response.roll,
            response.pitch,
            response.yaw,
            response.l_hip,
            response.l_knee,
            response.r_hip,
            response.r_knee,
            response.l_shoulder,
            response.l_elbow,
            response.r_shoulder,
            response.r_elbow,
            response.at_goal,
          ))

        break


def main(args=None):
  rclpy.init(args=args)

  executor = MultiThreadedExecutor(num_threads=3)
  serial_tester = SerialTest()

  executor.add_node(serial_tester)
  executor.spin()
  executor.shutdown()

  serial_tester.destroy_node()
  # rclpy.spin(serial_tester)
  rclpy.shutdown()

if __name__ == '__main__':
  main()