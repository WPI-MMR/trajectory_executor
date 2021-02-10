import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json


class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        self.publisher_ = self.create_publisher(
            String,
            'joint_angles',
            10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        dummy_jnt_values = { 
            "FL_HFE": 3.9571158441228693,
            "FL_KFE": -3.705995755504008, 
            "FL_ANKLE": 0.0, 
            "FR_HFE": -6.240330487904615, 
            "FR_KFE": 4.285736406870781, 
            "FR_ANKLE": 0.0, 
            "HL_HFE": -0.32403276755326144, 
            "HL_KFE": -5.4244141069928276, 
            "HL_ANKLE": 0.0, 
            "HR_HFE": -4.428453979549971, 
            "HR_KFE": -2.745596111102513, 
            "HR_ANKLE": 0.0
        }
        json_string = json.dumps(dummy_jnt_values)
        msg = String()
        msg.data = "ACTION" + json_string 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    rclpy.spin(planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()