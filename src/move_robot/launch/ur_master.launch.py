from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_share_dir = get_package_share_directory('move_robot')
    gcode_file = os.path.join(package_share_dir, 'gcode', 'six_cube.gcode')

    file_launch_arg = DeclareLaunchArgument(
        'file',
        default_value=gcode_file,
        description='Full path to the G-code file to be executed'
    )

    move_ur_node = Node(
        package='move_robot',
        executable='move_ur',
        name='move_ur_node',
        output='screen'
    )

    keyboard_node = Node(
        package='move_robot',
        executable='keyboard_node',
        name='keyboard_node',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',
    )

    gcode_interpreter_node = Node(
        package='move_robot',
        executable='gcode_interpreter',
        name='gcode_interpreter',
        output='screen',
        parameters=[
            {'x_offset': -935.0,
            'y_offset': -250.0,
            'z_offset': 199.8,
            'print_speed_multiplier': 3.0,
            'extrusion_scale_factor': 1.0,
            'wrist_angle': 90.0},
            {'file': LaunchConfiguration('file')}
        ]
    )

    delayed_gcode_node = TimerAction(
        period=4.0,
        actions=[gcode_interpreter_node]
    )

    return LaunchDescription([
        file_launch_arg,
        move_ur_node,
        keyboard_node,
        delayed_gcode_node
    ])
