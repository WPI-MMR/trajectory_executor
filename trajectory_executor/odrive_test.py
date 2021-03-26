import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest

import serial
import random


class OdriveTest(Node):
  """Tests sending a setpoint to the arduino/odrive and receiving tracking information"""
  def __init__(self):
    super().__init__('odrive_text')

    self.publisher = self.create_publisher(
      JointAngles,
      'joint_angles',
      10)

    self.srv_client = self.create_client(
      SensorDataRequest,
      'sensor_data_request')


def main(args=None):
  rclpy.init(args=args)
  odrive_tester = OdriveTest()

  movements = [] # list of joint angles

  angles = JointAngles()
  angles.left_hip = random.randint(0, 359)
  angles.left_knee = random.randint(0, 359)
  angles.right_hip = random.randint(0, 359)
  angles.right_knee = random.randint(0, 359)
  angles.left_shoulder = random.randint(0, 359)
  angles.left_elbow = random.randint(0, 359)
  angles.right_shoulder = random.randint(0, 359)
  angles.right_elbow = random.randint(0, 359)

  odrive_tester.get_logger().info("Sending joint angles")
  odrive_tester.publisher.publish(angles)
  rclpy.spin_once(odrive_tester)
  odrive_tester.destroy_node()
  rclpy.shutdown()

  # while rclpy.ok():
  #   rclpy.spinOnce(odrive_tester)


if __name__ == "__main__":
  main()