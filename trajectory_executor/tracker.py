import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, JointAngles as JA_MSG
from trajectory_interfaces.srv import SensorDataRequest, JointAngles as JA_SRV

import serial
import time


class TrajectoryTracker(Node):
  """Trajectory tracking for an 8-DOF quadruped robot given a trajectory"""
  def __init__(self):
    super().__init__('trajectory_tracker')
    # self.sd_publisher = self.create_publisher(
    #   SensorData,
    #   'traj_start_data',
    #   10)

    # self.ja_publisher = self.create_publisher(
    #   JointAngles,
    #   'joint_angles',
    #   10)

    self.ja_client = self.create_client(
      JA_SRV, 
      'joint_angles')

    self.srv_client = self.create_client(
      SensorDataRequest,
      'sensor_data_request')

    self.subscription = self.create_subscription(
      Trajectory,
      'new_traj',
      self.new_traj_callback,
      10)

    while not self.srv_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info("Serial service inactive, waiting...")

    self.req = SensorDataRequest.Request()
    self.traj = Trajectory()
    self.traj_queued = False

    # timer_period = 2 # seconds
    # self.timer = self.create_timer(timer_period, self.timer_callback)

  def send_request(self):
    self.req.requested = True
    self.future = self.srv_client.call_async(self.req)

  def send_ja(self, ja_req: JA_SRV.Request):
    self.future = self.ja_client.call_async(ja_req)

  def new_traj_callback(self, msg: Trajectory):
    """Receive trajectory from generator node"""
    self.get_logger().info("Received trajectory from generator")
    self.traj = msg
    self.traj_queued = True


def main(args=None):
  rclpy.init(args=args)
  trajectory_tracker = TrajectoryTracker()

  trajectory_tracker.get_logger().info("Ready to receive new trajectory")
  while rclpy.ok():
    rclpy.spin_once(trajectory_tracker)
    if trajectory_tracker.traj_queued:
      trajectory_tracker.get_logger().info("Executing...")
      ja_req = JA_SRV.Request()
      for ja in trajectory_tracker.traj.traj:
        goal = False
        ja_req.ja = ja
        trajectory_tracker.send_ja(ja_req)
        trajectory_tracker.get_logger().info(f"R_SH: {ja.right_shoulder}")
        trajectory_tracker.get_logger().info(f"R_EL: {ja.right_elbow}")

        while rclpy.ok():
          rclpy.spin_once(trajectory_tracker)
          if trajectory_tracker.future.done():
            try:
              response = trajectory_tracker.future.result()
            except Exception as e:
              trajectory_tracker.get_logger().info(f"Failed to get ack: {e}")
            else:
              if response.setpoint_ack == 1:
                trajectory_tracker.get_logger().info("Ack reported success")
                break
              elif response.setpoint_ack == 2:
                trajectory_tracker.get_logger().info("Ack reported malformed packet... Retrying")
                trajectory_tracker.send_ja(ja_req)
              else:
                trajectory_tracker.get_logger().info("Something went wrong...")

        # time.sleep(0.1)
        while rclpy.ok():
          trajectory_tracker.get_logger().info("Waiting for move complete...")
          trajectory_tracker.send_request()
          while rclpy.ok():
            rclpy.spin_once(trajectory_tracker)
            if trajectory_tracker.future.done():
              try:
                response = trajectory_tracker.future.result()
              except Exception as e:
                trajectory_tracker.get_logger().info(f"Service call failed {e}")
              else:
                if response.at_goal:
                  goal = True
                break

          if goal:
            trajectory_tracker.get_logger().info("Goal reached")
            break

      trajectory_tracker.traj_queued = False # reset to wait for new traj

  # rclpy.spin(trajectory_tracker)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
