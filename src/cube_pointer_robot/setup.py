from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cube_pointer_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[
        package_name,
        'camera_node',
        'robot_control',
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', glob('bringup/launch/*.py')),
        # Include config files
        ('share/' + package_name + '/config', glob('bringup/config/*.yaml')),
        ('share/' + package_name + '/camera_config', glob('camera_node/config/*.yaml')),
        ('share/' + package_name + '/robot_config', glob('robot_control/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel-hatlem',
    maintainer_email='daniehn@ntnu.no',
    description='Cube pointing robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_node.camera_node:main',
            'robot_controller = robot_control.robot_controller:main',
            'box_detection_node = camera_node.box_detection_node:main',
            'coordinator_test_node = camera_node.coordinator_test_node:main',
        ],
    },
)
