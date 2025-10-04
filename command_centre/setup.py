from setuptools import find_packages, setup

package_name = 'command_centre'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/command_centre.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'psutil',  # Required for process management in rover_command_centre
    ],
    zip_safe=True,
    maintainer='ByteSizedRobotics',
    maintainer_email='your.email@example.com',
    description='Command Centre and Communication Hub Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_command_centre = command_centre.rover_command_centre:main',
        ],
    },
)
