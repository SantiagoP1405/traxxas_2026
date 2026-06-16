#version 2 — reordenado, con watchdog y pure pursuit (t+3s)
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
    args = [
        # -- Camara ZED -----------------------------------------------
        DeclareLaunchArgument('color_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic',
            default_value='/zed/zed_node/depth/depth_registered'),
 
        # -- Señal de alto: deteccion ---------------------------------
        DeclareLaunchArgument('min_area_detect', default_value='200.0'),
        DeclareLaunchArgument('debug_vision',    default_value='true'),
 
        # -- Señal de alto: confirmacion ------------------------------
        DeclareLaunchArgument('sign_score_gain',    default_value='2'),
        DeclareLaunchArgument('sign_score_decay',   default_value='1'),
        DeclareLaunchArgument('sign_score_max',     default_value='10'),
        DeclareLaunchArgument('sign_score_trigger', default_value='5'),
        DeclareLaunchArgument('sign_min_area_conf', default_value='400.0'),
        DeclareLaunchArgument('sign_strong_area',   default_value='5000.0'),
        DeclareLaunchArgument('sign_max_dist_conf', default_value='0.65'),
 
        # -- Paso peatonal: deteccion ROI -----------------------------
        DeclareLaunchArgument('roi_top_frac',    default_value='0.65',
            description='Fraccion superior del ROI (0.0-1.0)'),
        DeclareLaunchArgument('roi_bottom_frac', default_value='1.00'),
        DeclareLaunchArgument('roi_left_frac',   default_value='0.20',
            description='Fraccion izquierda del ROI (excluye lineas carril)'),
        DeclareLaunchArgument('roi_right_frac',  default_value='0.80'),
        DeclareLaunchArgument('min_stripes',     default_value='3',
            description='Franjas blancas minimas para detectar'),
        DeclareLaunchArgument('min_stripe_width', default_value='3'),
        DeclareLaunchArgument('max_stripe_width', default_value='40'),
        DeclareLaunchArgument('white_threshold',  default_value='180'),
        DeclareLaunchArgument('min_fill_ratio',   default_value='0.50'),
 
        # -- Paso peatonal: confirmacion ------------------------------
        DeclareLaunchArgument('cross_score_gain',    default_value='2'),
        DeclareLaunchArgument('cross_score_decay',   default_value='1'),
        DeclareLaunchArgument('cross_score_max',     default_value='10'),
        DeclareLaunchArgument('cross_score_trigger', default_value='5'),
        DeclareLaunchArgument('cross_min_stripes',   default_value='3.0'),
        DeclareLaunchArgument('cross_max_dist',      default_value='1.20',
            description='Distancia maxima para confirmar paso peatonal [m]'),
 
        # -- PWM throttle ---------------------------------------------
        DeclareLaunchArgument('throttle_center', default_value='2457'),
        DeclareLaunchArgument('throttle_max',    default_value='2600'),
        DeclareLaunchArgument('throttle_min',    default_value='1638'),
 
        # -- PWM direccion --------------------------------------------
        DeclareLaunchArgument('dir_center',    default_value='2642'),
        DeclareLaunchArgument('dir_max_right', default_value='3276'),
        DeclareLaunchArgument('dir_min_left',  default_value='1669'),
        DeclareLaunchArgument('max_steer_rad', default_value='0.349'), # rad maximo de giro (20° aprox),
 
        # -- Pure Pursuit ---------------------------------------------
        DeclareLaunchArgument('path_csv',      default_value=''),
        DeclareLaunchArgument('trayectoria',   default_value='/home/movimiento/Workspaces/traxxas/src/traxxas_pose_estimation/trayectorias/traxxas_waypoints_1.csv',
                              description='Ruta al CSV con waypoints'),
        DeclareLaunchArgument('circle_radius', default_value='0.8'),
        DeclareLaunchArgument('circle_points', default_value='300'),
        DeclareLaunchArgument('lookahead',     default_value='0.15'),
        DeclareLaunchArgument('k_gain',        default_value='0.5'),
        DeclareLaunchArgument('v_ref',         default_value='0.5'),
        DeclareLaunchArgument('wheelbase',     default_value='0.335'),
 
        # -- Watchdog -------------------------------------------------
        DeclareLaunchArgument('watchdog_timeout', default_value='0.3'),
    ]
 
    # =================================================================
    #  NODO 1 — DETECCION SEÑAL DE ALTO
    # =================================================================
    stop_sign_node = Node(
        package='traxxas_stop',
        executable='stop_sign_node',
        name='stop_sign_detector',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'min_area':    LaunchConfiguration('min_area_detect'),
            'debug':       LaunchConfiguration('debug_vision'),
        }]
    )
 
    # =================================================================
    #  NODO 2 — CONFIRMACION SEÑAL DE ALTO
    # =================================================================
    stop_confirmation_node = Node(
        package='traxxas_stop',
        executable='stop_confirmation_node',
        name='stop_confirmation',
        output='screen',
        parameters=[{
            'score_gain':                    LaunchConfiguration('sign_score_gain'),
            'score_decay':                   LaunchConfiguration('sign_score_decay'),
            'score_max':                     LaunchConfiguration('sign_score_max'),
            'score_trigger':                 LaunchConfiguration('sign_score_trigger'),
            'min_area_for_confirmation':     LaunchConfiguration('sign_min_area_conf'),
            'strong_area_for_confirmation':  LaunchConfiguration('sign_strong_area'),
            'max_distance_for_confirmation': LaunchConfiguration('sign_max_dist_conf'),
        }]
    )
 
    # =================================================================
    #  NODO 3 — DETECCION PASO PEATONAL
    # =================================================================
    crosswalk_node = Node(
        package='traxxas_stop',
        executable='crosswalk_node',
        name='crosswalk_detector',
        output='screen',
        parameters=[{
            'color_topic':      LaunchConfiguration('color_topic'),
            'depth_topic':      LaunchConfiguration('depth_topic'),
            'roi_top_frac':     LaunchConfiguration('roi_top_frac'),
            'roi_bottom_frac':  LaunchConfiguration('roi_bottom_frac'),
            'roi_left_frac':    LaunchConfiguration('roi_left_frac'),
            'roi_right_frac':   LaunchConfiguration('roi_right_frac'),
            'min_stripes':      LaunchConfiguration('min_stripes'),
            'min_stripe_width': LaunchConfiguration('min_stripe_width'),
            'max_stripe_width': LaunchConfiguration('max_stripe_width'),
            'white_threshold':  LaunchConfiguration('white_threshold'),
            'min_fill_ratio':   LaunchConfiguration('min_fill_ratio'),
            'debug':            LaunchConfiguration('debug_vision'),
        }]
    )
 
    # =================================================================
    #  NODO 4 — CONFIRMACION PASO PEATONAL
    # =================================================================
    crosswalk_confirmation_node = Node(
        package='traxxas_stop',
        executable='crosswalk_confirmation_node',
        name='crosswalk_confirmation',
        output='screen',
        parameters=[{
            'score_gain':                      LaunchConfiguration('cross_score_gain'),
            'score_decay':                     LaunchConfiguration('cross_score_decay'),
            'score_max':                       LaunchConfiguration('cross_score_max'),
            'score_trigger':                   LaunchConfiguration('cross_score_trigger'),
            'min_stripes_for_confirmation':    LaunchConfiguration('cross_min_stripes'),
            'max_distance_for_confirmation':   LaunchConfiguration('cross_max_dist'),
        }]
    )
 
    # =================================================================
    #  NODO 5 — MERGER (fusiona stop_sign + crosswalk -> stop_event)
    # =================================================================
    merger_node = Node(
        package='traxxas_stop',
        executable='stop_event_merger_node',
        name='stop_event_merger',
        output='screen',
    )
 
    # =================================================================
    #  NODO 6 — ACTION SERVER
    #  Suscribe a /traxxas/stop_event/* (ya no a stop_sign directamente)
    # =================================================================
    stop_routine_server = Node(
        package='traxxas_stop',
        executable='stop_routine_server',
        name='stop_routine_action_server',
        output='screen',
    )
 
    # =================================================================
    #  NODO 7 — WATCHDOG
    # =================================================================
    watchdog_node = Node(
        package='traxxas_stop',
        executable='traxxas_watchdog_node',
        name='traxxas_watchdog',
        output='screen',
        parameters=[{
            'timeout':         LaunchConfiguration('watchdog_timeout'),
            'throttle_center': LaunchConfiguration('throttle_center'),
            'throttle_min':    LaunchConfiguration('throttle_min'),
            'dir_center':      LaunchConfiguration('dir_center'),
        }]
    )
 
    # =================================================================
    #  NODO 8 — PURE PURSUIT (t+3s)
    # =================================================================
    pp_delayed = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[launch] t+3s: arrancando Pure Pursuit...'),
            Node(
                package='traxxas_stop',
                executable='pure_pursuit_stop_traxxas',
                name='pure_pursuit_traxxas',
                output='screen',
                parameters=[{
                    'path_csv':        LaunchConfiguration('path_csv'),
                    #'path_csv':        LaunchConfiguration('trayectoria'),
                    'circle_radius':   LaunchConfiguration('circle_radius'),
                    'circle_points':   LaunchConfiguration('circle_points'),
                    'lookahead':       LaunchConfiguration('lookahead'),
                    'k_gain':          LaunchConfiguration('k_gain'),
                    'v_ref':           LaunchConfiguration('v_ref'),
                    'wheelbase':       LaunchConfiguration('wheelbase'),
                    'throttle_center': LaunchConfiguration('throttle_center'),
                    'throttle_max':    LaunchConfiguration('throttle_max'),
                    'throttle_min':    LaunchConfiguration('throttle_min'),
                    'dir_center':      LaunchConfiguration('dir_center'),
                    'dir_max_right':   LaunchConfiguration('dir_max_right'),
                    'dir_min_left':    LaunchConfiguration('dir_min_left'),
                    'max_steer_rad':   LaunchConfiguration('max_steer_rad'),
                }]
            ),
        ]
    )
 
    return LaunchDescription(
        args + [
            stop_sign_node,             # deteccion señal de alto
            stop_confirmation_node,     # score señal de alto
            crosswalk_node,             # deteccion paso peatonal
            crosswalk_confirmation_node,# score paso peatonal
            merger_node,                # fusiona -> /stop_event/*
            stop_routine_server,        # action server FSM
            watchdog_node,              # seguridad
            pp_delayed,                 # pure pursuit t+3s
        ]
    )