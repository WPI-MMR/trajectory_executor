import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles, Trajectory
from trajectory_interfaces.srv import SensorDataRequest

import serial
import time


class TrajectoryTracker(Node):
  """Trajectory tracking for an 8-DOF quadruped robot given a trajectory"""
  def __init__(self):
    super().__init__('trajectory_tracker')
    self.ja_pub = self.create_publisher(
      JointAngles,
      'joint_angles',
      10)

    self.traj_sub = self.create_subscription(
      Trajectory,
      'trajectory',
      self.traj_callback,
      10)

    self.srv_client = self.create_client(
      SensorDataRequest,
      'sensor_data_request')

    while not self.srv_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info("Serial service inactive, waiting...")

    self.req = SensorDataRequest.Request()
    self.req.requested = True
    self.traj_available = False

  def send_data_request(self):
    # self.req.requested = True
    self.future = self.srv_client.call_async(self.req)

  def traj_callback(self, traj: Trajectory):
    """Receive trajectory from generator node"""
    self.get_logger().info("Trajectory received. Executing...")
    self.traj = traj
    self.traj_available = True
    # for ja in traj.trajectory:
    #   self.ja_pub.publish(ja)
    #   goal = False

    #   while rclpy.ok():
    #     self.get_logger().info("spin")
    #     self.send_data_request()
    #     while rclpy.ok():
    #       self.get_logger().info("future")
    #       rclpy.spin_once(self)
    #       if self.future.done():
    #         try:
    #           self.get_logger().info("response")
    #           response = self.future.result()
    #         except Exception as e:
    #           self.get_logger().warn(f"Service call failed {e}")
    #         else:
    #           if response.at_goal:
    #             goal = True
    #           break
    #     if goal:
    #       break

  # def spin(self):
  #   while rclpy.ok():
  #     rclpy.spin_once(self)
  #     if self.traj_available:
  #       self.traj_available = False
  #       for ja in self.traj.trajectory:
  #         self.ja_pub.publish(ja)
  #         goal = False

  #         while rclpy.ok():
  #           self.send_data_request()
  #           while rclpy.ok():
  #             rclpy.spin_once(self)
  #             if self.future.done():
  #               try:
  #                 response = self.future.done()
  #               except Exception as e:
  #                 self.get_logger().warn(f"Service call failed {e}")
  #               else:
  #                 if response.at_goal:
  #                   goal = True
  #                 break
  #           if goal:
  #             break



def main(args=None):
  rclpy.init(args=args)
  trajectory_tracker = TrajectoryTracker()

  trajectory_tracker.get_logger().info("Ready")
  # rclpy.spin(trajectory_tracker)
  # while rclpy.ok():
  #   rclpy.spin_once(trajectory_tracker)

  while rclpy.ok():
    rclpy.spin_once(trajectory_tracker)
    if trajectory_tracker.traj_available:
      trajectory_tracker.traj_available = False
      for ja in trajectory_tracker.traj.trajectory:
        time.sleep(0.5)
        trajectory_tracker.ja_pub.publish(ja)
        # trajectory_tracker.get_logger().info(f"Goal: {ja.left_hip}")
        goal = False

        while rclpy.ok():
          # response = trajectory_tracker.srv_client.call(trajectory_tracker.req)
          # rclpy.spin_once(trajectory_tracker)
          # if response.at_goal:
          #   break
          trajectory_tracker.send_data_request()
          while rclpy.ok():
            rclpy.spin_once(trajectory_tracker)
            if trajectory_tracker.future.done():
              try:
                response = trajectory_tracker.future.result()
              except Exception as e:
                self.get_logger().warn(f"Service call failed {e}")
              else:
                # trajectory_tracker.get_logger().info(f"L_HIP: {response.l_hip}")
                if response.at_goal:
                  trajectory_tracker.future = None
                  goal = True
                break
          if goal:
            goal = False
            break

  # trajectory_tracker.spin()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
