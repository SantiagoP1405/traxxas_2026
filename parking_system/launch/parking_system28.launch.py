from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- RUTAS DE LOS PAQUETES ---
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
    traxxas_pose_launch_path = os.path.join(get_package_share_directory('traxxas_pose_estimation'), 'launch')
    
    # --- ARGUMENTOS ---
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port', 
        default_value= '/dev/rplidar',
        description='To select the serial port for the RPLIDAR'
    )

    # --- INCLUSIÓN DE OTROS LAUNCH FILES ---
    # 1. RPLidar
    lidar_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            rplidar_launch_path,
            '/rplidar_a3_launch.py'
        ]), launch_arguments={'serial_port': LaunchConfiguration('lidar_serial_port')}.items()
    )

    # 2. IMU, Odometría y control Serial (CORREGIDO)
    traxxas_pose_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            traxxas_pose_launch_path,
            '/bringup.launch.py'  # <--- CORREGIDO (antes tenía un guion bajo)
        ])
    )

    # --- NODOS DEL SISTEMA DE ESTACIONAMIENTO ---
    fsm_parking_node = Node(
        package="parking_system",
        executable="fsm_parking3",
        parameters=[{}]
    )

    lidar_processor_node = Node(
        package="parking_system",
        executable="lidar_processor",
        parameters=[{}]
    )

    parking_controller_traxxas_node = Node(
        package="parking_system",
        executable="parking_controller_28", 
        parameters=[{
            'serial_port': '/dev/trx_ultrasonicos', 
        }]
    )

    # --- NODO DE VISIÓN (LANE DETECTOR) ---
    lane_detector_node = Node(
        package="parking_system", 
        executable="lane_detector_camara_28", 
        name="lane_detector_camara_28",
        output="screen"
    )

    return LaunchDescription([
        # El Event Handler del profe
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=lidar_processor_node,
                on_exit=[lidar_launcher]
            )
        ),
        
        # Argumentos
        lidar_serial_port_arg,
        
        # Launches externos
        lidar_launcher,
        traxxas_pose_launcher,  # <-- IMU y Odometría
        
        # Nodos propios
        lane_detector_node,     # <-- Cámara ZED
        lidar_processor_node,   
        parking_controller_traxxas_node,
        fsm_parking_node
    ])