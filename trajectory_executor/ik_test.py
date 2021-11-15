import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest

import serial
import random
import time
import math

from trajectory_executor.Parameters import *

class IK_test(Node):
  """Testing for IK"""
  def __init__(self):
    super().__init__('ik_test')

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
  ik_tester = IK_test()

  angles = JointAngles()

  while rclpy.ok():
    x_in = int(input("Enter x coord: "))
    y_in = int(input("Enter y coord: "))

    ik_angles = ik_solver(x_in, y_in)
    print(ik_angles)

    angles.right_shoulder = int(ik_angles[0])
    angles.right_elbow = int(ik_angles[1])
    angles.left_hip = 0
    angles.left_knee = 0
    angles.right_hip = 0
    angles.right_knee = 0
    angles.left_shoulder = 0
    angles.left_elbow = 0

    ik_tester.get_logger().info("Sending joint angles")
    ik_tester.get_logger().info(f"R Shoulder: {angles.right_shoulder}")
    ik_tester.get_logger().info(f"R Elbow: {angles.right_elbow}")

    ik_tester.publisher.publish(angles)

    ik_tester.get_logger().info("Tracking")
    goal = False
    while rclpy.ok():
      ik_tester.send_request()

      while rclpy.ok():
        rclpy.spin_once(ik_tester)
        if ik_tester.future.done():
          try:
            response = ik_tester.future.result()
          except Exception as e:
            ik_tester.get_logger().info(f"Service call failed {e}")
          else:
            # ik_tester.get_logger().info(f"R_SHOULDER: {response.r_shoulder}")
            # ik_tester.get_logger().info(f"R_ELBOW: {response.r_elbow}")
            if response.at_goal:
              goal = True
            break

      if goal:
        ik_tester.get_logger().info("Goal reached")
        break

  ik_tester.destroy_node()
  rclpy.shutdown()


def ik_solver(x, y):
  p = x**2 + y**2 - l12**2 - l22**2
  q = 2 * l12 * l22

  theta22 = -math.acos(p / q)

  k = (l22 * math.sin(theta22)) / (l12 + l22 * math.cos(theta22))
  theta12 = math.atan(y / x) - math.atan(k)

  theta12 = theta12 * 180 / math.pi
  theta22 = theta22 * 180 / math.pi

  if theta12 < 0:
    theta12 += 360
  if theta22 < 0:
    theta22 += 360

  return (theta12, theta22)
