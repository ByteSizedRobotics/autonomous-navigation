from setuptools import setup

package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py',
                                               'launch/nav2.launch.py',
                                               'launch/sensors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Louis Marleau',
    maintainer_email='lmarl090@uottawa.ca',
    description='ROS2 Autonomous navigation package using Nav2',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_json = my_rover_nav.cmdvel_to_json:main',
            'gps_waypoint_client = my_rover_nav.gps_waypoint_client:main',
        ],
    },
)

