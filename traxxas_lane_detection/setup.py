from setuptools import find_packages, setup

package_name = 'traxxas_lane_detection'

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
    maintainer='traxxas',
    maintainer_email='traxxas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'lane_detector_yolo = traxxas_lane_detection.lane_detector_yolo:main',
        'jetson_transmisor = traxxas_lane_detection.jetson_transmisor:main',
        'jetson_transmisor_point_cloud = traxxas_lane_detection.jetson_transmisorWithPointCloud:main',
        'lane_detector_yolo_external = traxxas_lane_detection.lane_detector_yolo_external:main',
        'lane_detector_pure_pursuit = traxxas_lane_detection.lane_detector_purePursuit:main
        ],
    },
)
