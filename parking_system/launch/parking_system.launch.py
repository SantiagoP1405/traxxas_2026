from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
    
    # Declare launch arguments for saving video
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port', 
        default_value= '/dev/rplidar',
        description='To select the serial port for the RPLIDAR'
    )

    lidar_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            rplidar_launch_path,
            '/rplidar_a2m12_launch.py'
        ]), launch_arguments={'serial_port': LaunchConfiguration('lidar_serial_port')}.items()
    )

    fsm_parking_node = Node(
        package="parking_system",
        executable="fsm_parking3",
        parameters=[{
            # Here you can add all the parameters that your node needs, for example:
            # 'param_name': LaunchConfiguration('param_name'),
        }]

    )


    lidar_processor_node = Node(
        package="parking_system",
        executable="lidar_processor",
        parameters=[{
            # Here you can add all the parameters that your node needs, for example:
            # 'param_name': LaunchConfiguration('param_name'),
        }]

    )

    parking_controller_traxxas_node = Node(
        package="parking_system",
        executable="fsm_parking3",
        parameters=[{
            'serial_port': '/dev/ttyUSB0', 
        }]

    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=lidar_processor_node,
                                        on_exit=[lidar_launcher])),
        lidar_serial_port_arg,
        lidar_launcher,
        parking_controller_traxxas_node,
        fsm_parking_node
    ])