from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_executor',
            node_namespace='com_socket',
            node_executable='socket_connection',
            node_name='sim'
        ),
    ])
