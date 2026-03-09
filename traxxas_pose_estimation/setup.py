from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'traxxas_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marml',
    maintainer_email='marml@todo.todo',
    description='TODO: Package description',
    license='Apache2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_sensors_node = traxxas_pose_estimation.serial_sensors_node:main',
            'steering_angle_node = traxxas_pose_estimation.steering_angle_node:main',
            'odom_traxxas_node = traxxas_pose_estimation.ackermann_odom_node:main',
            'pose_traxxas_node = traxxas_pose_estimation.pose_traxxas_node:main',
        ],
    },
)
