from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='IMU + ToF sensor node via TCA9548A mux',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_tof_node = robot_sensors.imu_tof_node:main',
        ],
    },
)
