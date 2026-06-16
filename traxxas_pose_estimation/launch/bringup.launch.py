from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------- Launch args --------
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    frame_id = LaunchConfiguration('frame_id')

    pose_topic = LaunchConfiguration('pose_topic')
    pose_frame_id = LaunchConfiguration('pose_frame_id')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    calibrate_yaw_on_start = LaunchConfiguration('calibrate_yaw_on_start')
    normalize_to_pi = LaunchConfiguration('normalize_to_pi')

    return LaunchDescription([

        # -------- Serial params --------
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('baud', default_value='115200'),

        # -------- Frames --------
        DeclareLaunchArgument('frame_id', default_value='base_link'),

        # -------- pose_traxxas params --------
        DeclareLaunchArgument('pose_topic', default_value='/pose_traxxas'),
        DeclareLaunchArgument('pose_frame_id', default_value='traxxas_pose'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.01'),
        DeclareLaunchArgument('calibrate_yaw_on_start', default_value='true'),
        DeclareLaunchArgument('normalize_to_pi', default_value='true'),

        # -------- Serial node --------
        Node(
            package='traxxas_pose_estimation',
            executable='serial_sensors_node',
            name='serial_sensors_node',
            output='screen',
            parameters=[{
                'port': port,
                'baud': baud,
                'frame_id': frame_id,
            }],
        ),

        # -------- Pose node --------
        Node(
            package='traxxas_pose_estimation',
            executable='pose_traxxas_node',
            name='pose_traxxas_node',
            output='screen',
            parameters=[{
                'twist_topic': '/wheel/twist',
                'imu_topic': '/imu/data',
                'pose_topic': pose_topic,
                'frame_id': pose_frame_id,
                'velocity_deadband': velocity_deadband,
                'calibrate_yaw_on_start': calibrate_yaw_on_start,
                'normalize_to_pi': normalize_to_pi,
            }],
        ),
    ])