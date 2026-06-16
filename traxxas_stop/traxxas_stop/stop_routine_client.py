#!/usr/bin/env python3
"""
Cliente de la action StopRoutine.
Lanza la misión de 3 señales de alto y reporta feedback en terminal.

Uso
---
  # Terminal 1: lanzar todo el stack (sensores + detección + pp_node)
  ros2 launch traxxas_stop bringup_stop.launch.py

  # Terminal 2: correr el action server
  ros2 run traxxas_stop stop_routine_action_server

  # Terminal 3: enviar el goal con este cliente
  ros2 run traxxas_stop stop_routine_client

O bien ajustar los parámetros del goal directamente aquí abajo.
"""

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from traxxas_stop_interfaces.action import StopRoutine


class StopRoutineClient(Node):

    def __init__(self):
        super().__init__('stop_routine_client')
        self._client = ActionClient(self, StopRoutine, 'stop_routine')

    def send_goal(self):
        self.get_logger().info('Esperando action server...')
        self._client.wait_for_server()

        # ── Configurar la misión ──────────────────────────────
        goal = StopRoutine.Goal()
        goal.total_stops      = 3        # 3 señales de alto
        goal.cruise_pwm       = 2570.0   # PWM velocidad crucero
        goal.brake_pwm        = 2457.0   # PWM freno completo
        goal.prepare_dist_m   = 0.60     # Inicio de aproximación (m)
        goal.stop_dist_m      = 0.30     # Máximo 30 cm de la señal
        goal.wait_time_s      = 5.0      # Esperar 5 segundos detenido
        goal.brake_duration_s = 2.0      # Frenado gradual en 2 segundos
        goal.resume_duration_s = 1.5     # Arranque gradual en 1.5 segundos

        self.get_logger().info(
            f'Enviando goal: {goal.total_stops} paradas | '
            f'cruise={goal.cruise_pwm:.0f} | '
            f'stop_dist={goal.stop_dist_m:.2f} m'
        )

        send_goal_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado por el servidor.')
            return
        self.get_logger().info('Goal aceptado. Misión en progreso...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.state:12s}] '
            f'paradas={fb.stops_completed} | '
            f'PWM={fb.current_pwm:4d} | '
            f'dist={fb.distance_to_sign_m:.2f} m | '
            f'espera={fb.elapsed_wait_s:.1f} s'
        )

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f' Misión COMPLETADA: {result.stops_completed} paradas. '
                f'{result.message}'
            )
        else:
            self.get_logger().error(
                f' Misión FALLIDA: {result.message}'
            )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = StopRoutineClient()
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()


# ══════════════════════════════════════════════════════════════════════
#  INSTRUCCIONES DE INTEGRACIÓN
# ══════════════════════════════════════════════════════════════════════
#
#  1. ESTRUCTURA DE ARCHIVOS
#  ─────────────────────────
#  traxxas_stop/
#  ├── action/
#  │   └── StopRoutine.action          ← definición de la action
#  ├── traxxas_stop/
#  │   ├── stop_routine_action_server.py
#  │   ├── stop_routine_client.py      ← este archivo
#  │   ├── pp_node.py                  ← versión modificada
#  │   ├── stop_sign_node.py           ← sin cambios
#  │   └── stop_confirmation_node.py   ← sin cambios
#  ├── package.xml
#  └── setup.py
#
#
#  2. package.xml  – agregar estas líneas en <package>
#  ─────────────────────────────────────────────────────
#
#    <buildtool_depend>rosidl_default_generators</buildtool_depend>
#    <depend>action_msgs</depend>
#    <depend>rclpy</depend>
#    <member_of_group>rosidl_interface_packages</member_of_group>
#
#
#  3. setup.py  – reemplazar o unificar con el tuyo
#  ─────────────────────────────────────────────────
#
#  from setuptools import find_packages, setup
#  import os
#  from glob import glob
#
#  package_name = 'traxxas_stop'
#
#  setup(
#      name=package_name,
#      version='0.0.1',
#      packages=find_packages(exclude=['test']),
#      data_files=[
#          ('share/ament_index/resource_index/packages',
#              ['resource/' + package_name]),
#          ('share/' + package_name, ['package.xml']),
#          (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
#          # ← acción registrada para que colcon la compile
#          (os.path.join('share', package_name, 'action'), glob('action/*.action')),
#      ],
#      install_requires=['setuptools'],
#      zip_safe=True,
#      maintainer='marml',
#      maintainer_email='marml@todo.todo',
#      description='Traxxas stop mission with ROS2 Action',
#      license='TODO',
#      entry_points={
#          'console_scripts': [
#              'stop_sign_node         = traxxas_stop.stop_sign_node:main',
#              'stop_confirmation_node = traxxas_stop.stop_confirmation_node:main',
#              'stop_routine_server    = traxxas_stop.stop_routine_action_server:main',
#              'stop_routine_client    = traxxas_stop.stop_routine_client:main',
#              'pp_node                = traxxas_stop.pp_node:main',
#          ],
#      },
#  )
#
#
#  4. CMakeLists.txt  –  si el paquete es ament_cmake (no aplica si es
#     ament_python puro). Para paquete Python puro las actions se
#     declaran solo en package.xml + setup.py (ver arriba).
#
#     Si necesitas un paquete híbrido para generar las interfaces,
#     crea un paquete separado llamado traxxas_stop_interfaces con
#     CMakeLists.txt que declare la action, y desde traxxas_stop
#     depende de él:
#
#     traxxas_stop_interfaces/
#     ├── action/
#     │   └── StopRoutine.action
#     ├── CMakeLists.txt
#     └── package.xml
#
#     CMakeLists.txt mínimo:
#     ─────────────────────
#     cmake_minimum_required(VERSION 3.8)
#     project(traxxas_stop_interfaces)
#
#     find_package(ament_cmake REQUIRED)
#     find_package(rosidl_default_generators REQUIRED)
#     find_package(action_msgs REQUIRED)
#
#     rosidl_generate_interfaces(${PROJECT_NAME}
#       "action/StopRoutine.action"
#       DEPENDENCIES action_msgs
#     )
#
#     ament_package()
#
#     package.xml mínimo:
#     ──────────────────
#     <package format="3">
#       <name>traxxas_stop_interfaces</name>
#       <version>0.0.1</version>
#       <description>Interfaces for traxxas stop action</description>
#       <maintainer email="marml@todo.todo">marml</maintainer>
#       <license>TODO</license>
#       <buildtool_depend>ament_cmake</buildtool_depend>
#       <buildtool_depend>rosidl_default_generators</buildtool_depend>
#       <depend>action_msgs</depend>
#       <member_of_group>rosidl_interface_packages</member_of_group>
#       <export>
#         <build_type>ament_cmake</build_type>
#       </export>
#     </package>
#
#
#  5. BUILD
#  ────────
#  cd ~/ros2_ws
#  colcon build --packages-select traxxas_stop_interfaces traxxas_stop
#  source install/setup.bash
#
#
#  6. EJECUCIÓN (3 terminales)
#  ────────────────────────────
#  # T1: sensores + pose
#  ros2 launch traxxas_pose_estimation bringup.launch.py
#
#  # T2: detección de señal (camara ZED)
#  ros2 run traxxas_stop stop_sign_node
#  ros2 run traxxas_stop stop_confirmation_node
#
#  # T3: pure pursuit + action server (en orden)
#  ros2 run traxxas_stop pp_node
#  ros2 run traxxas_stop stop_routine_server
#
#  # T4: disparar la misión
#  ros2 run traxxas_stop stop_routine_client
#
#  # Monitoreo de feedback en tiempo real:
#  ros2 topic echo /traxxas/stop_sign/active
#  ros2 topic echo /throttle_motor