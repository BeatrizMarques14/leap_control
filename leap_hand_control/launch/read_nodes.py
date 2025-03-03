import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='leap_hand_control',
            executable='finger_positions', 
            name='finger_positions'
        ),
        launch_ros.actions.Node(
            package='leap_hand_control',
            executable='finger_velocities',  
            name='finger_velocities'
        ),
        launch_ros.actions.Node(
            package='leap_hand_control',
            executable='finger_currents',  
            name='finger_currents'
        )
    ])
