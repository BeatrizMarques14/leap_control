from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_control',
            executable='finger_positions', 
            name='finger_positions'
        ),
        Node(
            package='leap_control',
            executable='finger_velocities',  
            name='finger_velocities'
        ),
        Node(
            package='leap_control',
            executable='finger_currents',  
            name='finger_currents'
        )
    ])
