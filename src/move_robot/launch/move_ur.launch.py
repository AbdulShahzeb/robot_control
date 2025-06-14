from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='move_robot',
            executable='move_ur',
            name='move_ur_node',
            output='screen'
        )
    ])
