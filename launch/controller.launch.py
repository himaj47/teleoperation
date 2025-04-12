from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleoperation',
            namespace='',
            executable='gamepad_controller',
            name='gamepad_controller'
        ),
    ])