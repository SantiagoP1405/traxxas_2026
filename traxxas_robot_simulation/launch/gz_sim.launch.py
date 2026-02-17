from launch import LaunchDescription
from launch_ros.actions import Node  
from launch.actions import IncludeLaunchDescription  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch_ros.descriptions import ParameterValue  
from launch.substitutions import Command  
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory 

def generate_launch_description():
    # Get the robot description package path
    robot_description_pkg = get_package_share_path('traxxas_robot_description')

    # Get the Gazebo launch package
    gz_launch_path = os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch')

    # Get the urdf file path
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'traxxas.urdf.xacro')

    # Get the Rviz2 config file path
    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'config.rviz')

    # Get the Gazebo bridge config file path
    gz_bridge_config_path = os.path.join(get_package_share_directory('traxxas_robot_simulation'), 'config', 'gz_bridge.yaml')

    #Get the SDF world file path
    gz_world_path = os.path.join(get_package_share_path('traxxas_robot_simulation'),
        'worlds',
        'track_world.sdf')

    # Node for robot state publisher
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Include Gazebo launch file
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_launch_path,
            "/ign_gazebo.launch.py"
        ]), launch_arguments={'gz_args': f'{gz_world_path} -r'}.items()
    )

    # Node for robot generation in Gazebo
    spawn_robot_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-x', '-0.739400',
                   '-y', '3.035775',
                   '-Y', '3.14']  # Yaw in radians
    )

    # Node for rviz2 launch
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Node for Gazebo-ROS2 bridge
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file' : gz_bridge_config_path}]
    )

    # Node for PWM to cmd_vel conversion
    pwm_to_cmd_vel_node = Node(
        package = 'traxxas_cmd_sim',
        executable='cmd_sim',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim_launch,
        spawn_robot_node,
        gz_bridge_node,
        rviz2_node,
        pwm_to_cmd_vel_node,
    ])

