from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'traxxas_stop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marml',
    maintainer_email='marml@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'crosswalk_confirmation_node = traxxas_stop.crosswalk_confirmation_node:main',
            'crosswalk_node = traxxas_stop.crosswalk_node:main',
            'pp_node = traxxas_stop.pp_node:main',
            'pwm_controller = traxxas_stop.pwm_controller:main',
            'stop_confirmation_node = traxxas_stop.stop_confirmation_node:main',
            'stop_event_merger_node = traxxas_stop.stop_event_merger_node:main', 
            'stop_maneuvers = traxxas_stop.stop_maneuvers:main',
            'stop_routine_server = traxxas_stop.stop_routine_action_server:main',
            'stop_routine_client = traxxas_stop.stop_routine_client:main',
            'stop_sign_node = traxxas_stop.stop_sign_node:main',
            'traxxas_watchdog_node = traxxas_stop.traxxas_watchdog_node:main',
        ],
    },
)
