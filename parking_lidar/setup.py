from setuptools import find_packages, setup

package_name = 'parking_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanianwn',
    maintainer_email='tanianwn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_processor = parking_lidar.lidar_processor:main',
            'fsm_parking = parking_lidar.fsm_parking:main',
            'parking_controller = parking_lidar.parking_controller:main',
        ],
    },
)
