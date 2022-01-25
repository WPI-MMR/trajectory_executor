import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import Trajectory, JointAngles, DataRequest, DataResponse, AckResponse
# from trajectory_interfaces.srv import SensorDataRequest, JointAngles as JA_SRV

import serial
import time


class TrajectoryTracker(Node):
  """Trajectory tracking for an 8-DOF quadruped robot given a trajectory"""
  def __init__(self):
    super().__init__('trajectory_tracker')
    self.dr_publisher = self.create_publisher(
      DataRequest,
      'data_request',
      10)

    self.ja_publisher = self.create_publisher(
      JointAngles,
      'joint_angles',
      10)

    self.dr_subscription = self.create_subscription(
      DataResponse,
      'data_response',
      self.receive_data,
      10)

    self.ja_subscription = self.create_subscription(
      AckResponse,
      'ack_response',
      self.receive_ack,
      10)

    # self.ja_client = self.create_client(
    #   JA_SRV, 
    #   'joint_angles')

    # self.srv_client = self.create_client(
    #   SensorDataRequest,
    #   'sensor_data_request')

    self.subscription = self.create_subscription(
      Trajectory,
      'new_traj',
      self.new_traj_callback,
      10)

    # while not self.srv_client.wait_for_service(timeout_sec=1.0):
    #   self.get_logger().info("Serial service inactive, waiting...")

    # self.ja_req = JA_SRV.Request()
    # self.req = SensorDataRequest.Request()

    self.data_request = DataRequest()
    self.data_request.requested = True

    self.traj = Trajectory()
    self.traj_queued = False
    self.ack = AckResponse()
    self.ack_queued = False
    self.data = DataResponse()
    self.data_queued = False

    # timer_period = 2 # seconds
    # self.timer = self.create_timer(timer_period, self.timer_callback)

  # def send_request(self):
  #   self.req.requested = True
  #   self.future = self.srv_client.call_async(self.req)

  # def send_ja(self):
  #   self.future = self.ja_client.call_async(self.ja_req)

  def receive_ack(self, msg: AckResponse):
    self.ack = msg
    self.ack_queued = True

  def receive_data(self, msg: DataResponse):
    self.data = msg
    self.data_queued = True

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
      for ja in trajectory_tracker.traj.traj:
        goal = False
        trajectory_tracker.ja_publisher.publish(ja)
        # trajectory_tracker.ja_req.ja = ja
        # trajectory_tracker.send_ja()
        trajectory_tracker.get_logger().info(f"R_SH: {ja.right_shoulder}")
        trajectory_tracker.get_logger().info(f"R_EL: {ja.right_elbow}")

        while rclpy.ok():
          rclpy.spin_once(trajectory_tracker)
          if trajectory_tracker.ack_queued:
            trajectory_tracker.ack_queued = False
            if trajectory_tracker.ack.setpoint_ack == 1:
              trajectory_tracker.get_logger().info("Ack reported success")
              break
            else:
              trajectory_tracker.get_logger().info("Something went wrong... Retrying")
              trajectory_tracker.ja_publisher.publish(ja)
            # try:
            #   response = trajectory_tracker.future.result()
            # except Exception as e:
            #   trajectory_tracker.get_logger().info(f"Failed to get ack: {e}")
            # else:
            #   if response.setpoint_ack == 1:
            #     trajectory_tracker.get_logger().info("Ack reported success")
            #     break
            #   else:
            #     trajectory_tracker.get_logger().info("Something went wrong... Retrying")
            #     trajectory_tracker.send_ja()

        # time.sleep(0.1)
        while rclpy.ok():
          trajectory_tracker.get_logger().info("Waiting for move complete...")
          trajectory_tracker.dr_publisher.publish(trajectory_tracker.data_request)
          # trajectory_tracker.send_request()
          while rclpy.ok():
            rclpy.spin_once(trajectory_tracker)
            if trajectory_tracker.data_queued:
              trajectory_tracker.data_queued = False
              if trajectory_tracker.data.at_goal:
                goal = True
              break

            # if trajectory_tracker.future.done():
            #   try:
            #     response = trajectory_tracker.future.result()
            #   except Exception as e:
            #     trajectory_tracker.get_logger().info(f"Service call failed {e}")
            #   else:
            #     if response.at_goal:
            #       goal = True
            #     break

          if goal:
            trajectory_tracker.get_logger().info("Goal reached")
            break

      trajectory_tracker.traj_queued = False # reset to wait for new traj

  # rclpy.spin(trajectory_tracker)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
