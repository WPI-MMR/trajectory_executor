import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest

import serial
import random
import time


class OdriveTest(Node):
  """Tests sending a setpoint to the arduino/odrive and receiving tracking information"""
  def __init__(self):
    super().__init__('odrive_test')

    self.publisher = self.create_publisher(
      JointAngles,
      'joint_angles',
      10)

    self.srv_client = self.create_client(
      SensorDataRequest,
      'sensor_data_request')

    while not self.srv_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info("Serial service inactive, waiting...")

    self.req = SensorDataRequest.Request()

  def send_request(self):
    self.req.requested = True
    self.future = self.srv_client.call_async(self.req)


def main(args=None):
  rclpy.init(args=args)
  odrive_tester = OdriveTest()

  angles = JointAngles()

  while rclpy.ok():
    # angles.left_hip = random.randint(0, 359)
    # angles.left_knee = random.randint(0, 359)
    # angles.right_hip = random.randint(0, 359)
    # angles.right_knee = random.randint(0, 359)
    # angles.left_shoulder = random.randint(0, 359)
    # angles.left_elbow = random.randint(0, 359)
    angles.right_shoulder = random.randint(0, 359)
    angles.right_elbow = random.randint(0, 359)
    angles.left_hip = 0
    angles.left_knee = 0
    angles.right_hip = 0
    angles.right_knee = 0
    angles.left_shoulder = 0
    angles.left_elbow = 0
    # angles.right_shoulder = 0
    # angles.right_elbow = 0

    odrive_tester.get_logger().info("Sending joint angles")
    odrive_tester.publisher.publish(angles)

    odrive_tester.get_logger().info("Tracking")
    goal = False
    while rclpy.ok():
      odrive_tester.send_request()

      while rclpy.ok():
        rclpy.spin_once(odrive_tester)
        if odrive_tester.future.done():
          try:
            response = odrive_tester.future.result()
          except Exception as e:
            odrive_tester.get_logger().info(f"Service call failed {e}")
          else:
            odrive_tester.get_logger().info(f"L_HIP: {response.l_hip}")
            if response.at_goal:
              goal = True
            break

      if goal:
        odrive_tester.get_logger().info("Goal reached")
        break




  odrive_tester.destroy_node()
  rclpy.shutdown()



if __name__ == "__main__":
  main()