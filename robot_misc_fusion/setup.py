from setuptools import find_packages, setup

package_name = 'robot_misc_fusion'

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
    maintainer='nguye',
    maintainer_email='hobblingheli@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tof_publisher_node = robot_misc_fusion.tof_publisher_node:main',
            "tof_safety_node = robot_misc_fusion.tof_safety_node:main",
            'tof_to_scan_bridge = robot_misc_fusion.tof_to_scan_bridge:main',
            "person_detection_fusion_node = robot_misc_fusion.person_detection_fusion_node:main"
        ],
    },
)
