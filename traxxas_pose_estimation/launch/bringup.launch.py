from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------- Launch args --------
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    gear_ratio = LaunchConfiguration('gear_ratio')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheelbase = LaunchConfiguration('wheelbase')

    pwm_min = LaunchConfiguration('pwm_min')
    pwm_mid = LaunchConfiguration('pwm_mid')
    pwm_max = LaunchConfiguration('pwm_max')
    delta_max = LaunchConfiguration('delta_max')
    invert = LaunchConfiguration('invert')
    deadband_pwm = LaunchConfiguration('deadband_pwm')

    frame_id = LaunchConfiguration('frame_id')
    frame_odom = LaunchConfiguration('frame_odom')
    frame_base = LaunchConfiguration('frame_base')

    pose_topic = LaunchConfiguration('pose_topic')
    pose_frame_id = LaunchConfiguration('pose_frame_id')
    pose_publish_rate = LaunchConfiguration('pose_publish_rate')
    velocity_deadband = LaunchConfiguration('velocity_deadband')
    calibrate_yaw_on_start = LaunchConfiguration('calibrate_yaw_on_start')
    normalize_to_pi = LaunchConfiguration('normalize_to_pi')

    return LaunchDescription([
        # -------- Serial / vehicle params --------
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),

        DeclareLaunchArgument('gear_ratio', default_value='15.3'),
        DeclareLaunchArgument('wheel_radius', default_value='0.055'),
        DeclareLaunchArgument('wheelbase', default_value='0.335'),

        # -------- Steering PWM mapping --------
        DeclareLaunchArgument('pwm_min', default_value='1669'),
        DeclareLaunchArgument('pwm_mid', default_value='2642'),
        DeclareLaunchArgument('pwm_max', default_value='3276'),
        DeclareLaunchArgument('delta_max', default_value='0.436'),
        DeclareLaunchArgument('invert', default_value='false'),
        DeclareLaunchArgument('deadband_pwm', default_value='15'),

        # -------- Frames --------
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('frame_odom', default_value='odom'),
        DeclareLaunchArgument('frame_base', default_value='base_link'),

        # -------- pose_traxxas params --------
        DeclareLaunchArgument('pose_topic', default_value='/pose_traxxas'),
        DeclareLaunchArgument('pose_frame_id', default_value='traxxas_pose'),
        DeclareLaunchArgument('pose_publish_rate', default_value='50.0'),
        DeclareLaunchArgument('velocity_deadband', default_value='0.01'),
        DeclareLaunchArgument('calibrate_yaw_on_start', default_value='true'),
        DeclareLaunchArgument('normalize_to_pi', default_value='true'),

        # -------- Nodes --------
        Node(
            package='traxxas_pose_estimation',
            executable='serial_sensors_node',
            name='serial_sensors_node',
            output='screen',
            parameters=[{
                'port': port,
                'baud': baud,
                'gear_ratio': gear_ratio,
                'wheel_radius': wheel_radius,
                'frame_id': frame_id,
            }],
        ),

        Node(
            package='traxxas_pose_estimation',
            executable='steering_angle_node',
            name='steering_angle_node',
            output='screen',
            parameters=[{
                'topic_in': 'direction_servo',
                'topic_out': 'steering_angle',
                'pwm_min': pwm_min,
                'pwm_mid': pwm_mid,
                'pwm_max': pwm_max,
                'delta_max': delta_max,
                'invert': invert,
                'deadband_pwm': deadband_pwm,
            }],
        ),

        Node(
            package='traxxas_pose_estimation',
            executable='odom_traxxas_node',
            name='ackermann_odom_node',
            output='screen',
            parameters=[{
                'wheelbase': wheelbase,
                'gear_ratio': gear_ratio,
                'wheel_radius': wheel_radius,
                'frame_odom': frame_odom,
                'frame_base': frame_base,
            }],
        ),

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
                'publish_rate': pose_publish_rate,
                'velocity_deadband': velocity_deadband,
                'calibrate_yaw_on_start': calibrate_yaw_on_start,
                'normalize_to_pi': normalize_to_pi,
            }],
        ),
    ])