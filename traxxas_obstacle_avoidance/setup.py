from setuptools import find_packages, setup

package_name = 'traxxas_obstacle_avoidance'

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
    maintainer_email='dario5feb5b@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = traxxas_obstacle_avoidance.obstacle_avoidance_node:main',
            'see_obstacle_node = traxxas_obstacle_avoidance.see_obstacle_node:main'
        ],
    },
)
