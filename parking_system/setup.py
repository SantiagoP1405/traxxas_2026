from setuptools import setup

package_name = 'parking_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_parking = parking_system.fsm_parking:main',
            'fsm_parking2 = parking_system.fsm_parking2:main',
            'fsm_parking3 = parking_system.fsm_parking3:main',
            'lidar_processor = parking_system.lidar_processor:main',
            'parking_controller = parking_system.parking_controller:main',
            'test_motors = parking_system.test_motors:main',
            'test_sensors = parking_system.test_sensors:main',
            'test_freno = parking_system.test_freno:main',
            'correction_test= parking_system.correction_test:main',
            'parking_controller2 = parking_system.parking_controller2:main',
            'parking_controller3 = parking_system.parking_controller3:main',
            'parking_controller4 = parking_system.parking_controller4:main',
            'parking_controller5 = parking_system.parking_controller5:main',
            'parking_controller_traxxas = parking_system.parking_controller_traxxas:main',
        ],
    },
)
