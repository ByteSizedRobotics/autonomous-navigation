from setuptools import setup
import os
from glob import glob

package_name = 'csi_camera_stream'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Explicitly include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='bytesizedrobotics0@gmail.com',
    description='CSI Camera Stream Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['csi_camera_video = csi_camera_stream.csi_camera_video:main',
                            'csi_camera_inference = csi_camera_stream.csi_camera_inference:main',
                            'webRTC_publisher = csi_camera_stream.webRTC_publisher:main',
                            'usb_camera_video = csi_camera_stream.usb_camera_video:main',
                            'usb_webRTC_publisher = csi_camera_stream.usb_webRTC_publisher:main'],

    },
)
