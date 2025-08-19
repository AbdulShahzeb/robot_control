from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='move_robot',
            executable='calibrate_extruder',
            name='extruder_calibrator',
            output='screen',
            parameters=[{
                'num_runs': 5,
                'auto_increment': False,

                # Parameters used when auto_increment is True
                'base_distance': 50.0,
                'base_speed': 250.0,
                'speed_increment': 250.0,
            }]
        )
    ])
