from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    control_node = Node(
        package='traxxas_remote_control',
        executable='control_traxxas',
        name='joystick_microros_controller',
        output='screen'
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        control_node,
    ])
