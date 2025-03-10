from setuptools import find_packages, setup

package_name = 'potrider'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Louis Marleau',
    maintainer_email='lmarl090@uottawa.ca',
    description='ROS2 package for serial communication and WASD input',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_motor_node = potrider.serial_motor_node:main',
            'wasd_control = potrider.wasd_control:main',
        ],
    },
)
