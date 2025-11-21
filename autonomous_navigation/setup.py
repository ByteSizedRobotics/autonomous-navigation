from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple autonomous navigation with GPS waypoint following and LiDAR obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_navigation_gps = autonomous_navigation.autonomous_navigation_gps:main',
            'autonomous_navigation = autonomous_navigation.autonomous_navigation:main',
            'autonomous_navigation_fast_avoidance = autonomous_navigation.autonomous_navigation_fast_avoidance:main',
            'autonomous_navigation_with_goal = autonomous_navigation.autonomous_navigation_with_goal:main',
            'rover_serial_bridge = autonomous_navigation.rover_serial_bridge:main',
        ],
    },
)
