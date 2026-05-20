from setuptools import find_packages, setup

package_name = 'robot_nano_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'paho-mqtt'],
    zip_safe=True,
    maintainer='nguye',
    maintainer_email='hobblingheli@gmail.com',
    description='ROS2 node that bridges ROS2 topics and MQTT messages to an Arduino Nano via serial for robot peripheral control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_nano_bridge_node = robot_nano_bridge.robot_nano_bridge_node:main',
        ],
    },
)
