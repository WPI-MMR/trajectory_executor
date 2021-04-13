import rclpy
from rclpy.node import Node

from trajectory_interfaces.msg import JointAngles
from trajectory_interfaces.srv import SensorDataRequest
from trajectory_executor.algos.foward_wave import ForwardWave

from simple_term_menu import TerminalMenu


def _set_angle_FOR(angle):
  if angle < 0:
    angle = 360 + angle
  return int(angle)


class FowardWaveNode(Node, ForwardWave):
  def __init__(self, args, interpolation_steps: int = 5, 
              interpolation_wait: float = 0.005):
    rclpy.init(args=args)

    Node.__init__(self, 'trot')
    ForwardWave.__init__(self, interpolation_steps, interpolation_wait)

    # Create publisher
    self.joint_publisher = self.create_publisher(
      JointAngles, 'joint_angles', 10)

    # Create to ensure that the connection is up with com_serial. Not actually 
    # used.
    self.sensors_client = self.create_client(
      SensorDataRequest, 'sensor_data_request')

    while not self.sensors_client.wait_for_service(timeout_sec=1.):
      self.get_logger().warning('Serial service inactive, waiting...')

  def _send_angles(self):
    joint_msg = JointAngles()
    
    joint_msg.left_hip = _set_angle_FOR(self.joints['HL_HFE'])
    joint_msg.left_knee = _set_angle_FOR(self.joints['HL_KFE'])
    joint_msg.right_hip = _set_angle_FOR(self.joints['HR_HFE'])
    joint_msg.right_knee = _set_angle_FOR(self.joints['HR_KFE'])
    joint_msg.left_shoulder = _set_angle_FOR(self.joints['FL_HFE'])
    joint_msg.left_elbow = _set_angle_FOR(self.joints['FL_KFE'])
    joint_msg.right_shoulder = _set_angle_FOR(self.joints['FR_HFE'])
    joint_msg.right_elbow = _set_angle_FOR(self.joints['FR_KFE'])

    self.get_logger().debug('Sending the following robot configuration: {}'.format(self.joints))
    self.joint_publisher.publish(joint_msg)

  def close(self):
    self.destroy_node()
    rclpy.shutdown()


def wave(args=None):
  gait = FowardWaveNode(args, interpolation_steps=5, interpolation_wait=0.005)
  gait.run()
  gait.close()


def interactive(args=None):
  gait = FowardWaveNode(args, interpolation_steps=5, interpolation_wait=0.005)

  while True:
    options = (
      '[h] Home',
      '[j] Single Joint Control',
      '[a] Adjust Interpolation',
      '[x] Exit'
    )
    menu = TerminalMenu(options, title='Robot Interactive Control Menu')
    choice_idx = menu.show()

    if choice_idx == 0:
      gait.joints = {
        'FL_HFE': 0,
        'FL_KFE': 0,
        'FL_ANKLE': 0,
        'FR_HFE': 0,
        'FR_KFE': 0,
        'FR_ANKLE': 0,
        'HL_HFE': 0,
        'HL_KFE': 0,
        'HL_ANKLE': 0,
        'HR_HFE': 0,
        'HR_KFE': 0,
        'HR_ANKLE': 0,
      }
      gait.send_angles()

    elif choice_idx == 1:
      while True:
        key, _, value = input('Enter [JointID] [Value]: ').partition(' ')
        if key.lower() == 'all':
          for jkey in gait.joints.keys():
            gait.joints[jkey] = float(value)
        elif key.lower() == 'exit':
          break
        else:
          gait.joints[key] = float(value)
        gait.send_angles()

    elif choice_idx == 2:
      print(f'Current Interpolation Steps: {gait._inter_steps}')
      print(f'Current Interpolation Wait Time: {gait._inter_wait}')

      gait._inter_steps = int(input('New Interpolation Steps: '))
      gait._inter_wait = float(input('New Interpolation Wait Time: '))

    elif choice_idx == len(options) - 1:
      break